#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "i2cdev.h"
#include "esp_lcd_panel_vendor.h"
#include "pcf8574.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "sht30.h"

#include "espnow_config.h"
#include "sensor_config.h"

static const char *TAG = "controller";

#define I2C_BUS_PORT  0

static i2c_port_t i2c_port = I2C_NUM_0;
static uint32_t i2c_frequency = 100 * 1000;

static EventGroupHandle_t s_evt_group;

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DISPLAY_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define SDA_PIN           4
#define SCL_PIN           15
#define PIN_NUM_RST           -1
#define DISPLAY_I2C_HW_ADDR           0x3C

#define DISPLAY_LCD_H_RES              128
#define DISPLAY_LCD_V_RES              64

// Bit number used to represent command and parameter
#define DISPLAY_LCD_CMD_BITS           8
#define DISPLAY_LCD_PARAM_BITS         8

#define INPUT_I2C_HW_ADDR   0x22
#define OUTPUT_I2C_HW_ADDR   0x24

#define DELAY(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define DELAY_1S DELAY(1000)
#define DELAY_5S DELAY(5000)

#define TEMPERATURE_OPEN_LOUVRE 28
#define TEMPERATURE_CLOSE_LOUVRE 25

// greenhouse Kincony controller
#define STATION_ID 7

static i2c_dev_t pcf8574;

static void packet_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    ESP_LOGI(TAG, "packet sent, status is %d", status);
    assert(status == ESP_NOW_SEND_SUCCESS || status == ESP_NOW_SEND_FAIL);
    xEventGroupSetBits(s_evt_group, BIT(status));
}

static void init_espnow_slave(void)
{
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(MY_ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start() );
#if MY_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(MY_ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(packet_sent_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK) );

    // Alter this if you want to specify the gateway mac, enable encyption, etc
    const esp_now_peer_info_t broadcast_destination = {
        .peer_addr = MY_RECEIVER_MAC,
        .channel = MY_ESPNOW_CHANNEL,
        .ifidx = MY_ESPNOW_WIFI_IF
    };
    ESP_ERROR_CHECK( esp_now_add_peer(&broadcast_destination) );
}

static esp_err_t send_espnow_data(float temperature, float moisture, int voltage)
{
    const uint8_t destination_mac[] = MY_RECEIVER_MAC;
    static temphumind_data_t data;

    data.temperature = temperature;
    data.humidity = moisture;
    data.station = STATION_ID;
    data.voltage_mV = voltage;

    // Send it
    ESP_LOGI(TAG, "Sending %u bytes to %02x:%02x:%02x:%02x:%02x:%02x from station %d",
                 sizeof(data), destination_mac[0], destination_mac[1],destination_mac[2], destination_mac[3], destination_mac[4], destination_mac[5], STATION_ID);
    esp_err_t err = esp_now_send(destination_mac, (uint8_t*)&data, sizeof(data));
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error (%d)", err);
        return ESP_FAIL;
    }

    // Wait for callback function to set status bit
    EventBits_t bits = xEventGroupWaitBits(s_evt_group, BIT(ESP_NOW_SEND_SUCCESS) | BIT(ESP_NOW_SEND_FAIL), pdTRUE, pdFALSE, 2000 / portTICK_PERIOD_MS);
    if ( !(bits & BIT(ESP_NOW_SEND_SUCCESS)) )
    {
        if (bits & BIT(ESP_NOW_SEND_FAIL))
        {
            ESP_LOGE(TAG, "Send error");
            return ESP_FAIL;
        }
        ESP_LOGE(TAG, "Send timed out");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Sent!");
    return ESP_OK;
}

static void InitESPNow()
{
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

    ESP_LOGI(TAG, "Initializing ESPNOW");
    init_espnow_slave();

}

void displayTempHumidity(lv_disp_t *disp, float temperature, float humidity)
{
    // clear the display
    lv_obj_clean(lv_scr_act());
    
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    char buffer[200];
    sprintf(buffer, "Temperature %.1f\nHumidity %.1f", temperature, humidity);
    if (temperature > TEMPERATURE_OPEN_LOUVRE)
        sprintf(buffer, "Temperature %.1f\nHumidity %.1f\nlouvres opened", temperature, humidity);
    if (temperature <= TEMPERATURE_CLOSE_LOUVRE)
    sprintf(buffer, "Temperature %.1f\nHumidity %.1f\nlouvres closed", temperature, humidity);
    lv_label_set_text(label, buffer);
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    // lv_obj_set_width(label, disp->driver->hor_res);
    // lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}

void app_main(void)
{
    float temperature = 0, humidity = 0, prev_temperature = 0;
    sht30_t   sht30;

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = DISPLAY_I2C_HW_ADDR,
        .scl_speed_hz = DISPLAY_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = DISPLAY_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = DISPLAY_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = PIN_NUM_RST,
    };

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = DISPLAY_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = DISPLAY_LCD_H_RES * DISPLAY_LCD_V_RES,
        .double_buffer = true,
        .hres = DISPLAY_LCD_H_RES,
        .vres = DISPLAY_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    // initialize I2C for PCF8574
    ESP_LOGI(TAG, "Initialize I2C for PCF8574");

    i2c_device_config_t pfc8574_out_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OUTPUT_I2C_HW_ADDR,
        .scl_speed_hz = DISPLAY_LCD_PIXEL_CLOCK_HZ,
    };

    i2c_master_dev_handle_t pfc8574_out_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &pfc8574_out_cfg, &pfc8574_out_handle));

    i2c_device_config_t pfc8574_in_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INPUT_I2C_HW_ADDR,
        .scl_speed_hz = DISPLAY_LCD_PIXEL_CLOCK_HZ,
    };

    i2c_master_dev_handle_t pfc8574_in_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &pfc8574_in_cfg, &pfc8574_in_handle));

    InitESPNow();

    ESP_LOGI(TAG, "Initializing SHT30");

    // sht30.i2c_port = I2C_NUM_0;
    // sht30.addr = SHT30_ADDRESS;
    // if (sht30_init(&sht30) != ESP_OK)
    // {
    //     puts("Error initializing SHT30");
    // }

    // ESP_LOGI(TAG, "Getting temperature");
    // temperature = sht30_get_temperature(&sht30, true);
    // humidity = sht30_get_humidity(&sht30, false);

    sht30_init_add(i2c_bus, SHT30_ADDRESS_DEF, DISPLAY_LCD_PIXEL_CLOCK_HZ, MAX_WAIT_TIME);
    sht30_heater_control(&sht30, Heater_Disable);

    int isOpen = -1;
    uint8_t write_buf;
    // uint8_t read_buf;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 150;
    BaseType_t xWasDelayed;
    
    // before we start ensure the louvres are closed
    write_buf = 0xF9;
    ESP_LOGI(TAG, "Send Reverse motor direction (0x06) to PCF8574");
    ESP_ERROR_CHECK(i2c_master_transmit(pfc8574_out_handle, &write_buf, 1, -1));

    write_buf = 0xF8;
    ESP_LOGI(TAG, "Send Reverse motor enable (0x07) to PCF8574");
    ESP_ERROR_CHECK(i2c_master_transmit(pfc8574_out_handle, &write_buf, 1, -1));

    DELAY_5S;

    // get into a known state
    while (1)
    {
        sht30_single_shot(&sht30, Repeatability_High, ClockStretching_Disable);
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    
        prev_temperature = temperature;
        temperature = sht30_read_temperature_celsius(&sht30);
        humidity = sht30_read_humidity(&sht30);
        ESP_LOGI(TAG, "Temperature is %.2f celsius", temperature);
        ESP_LOGI(TAG, "Humidity is %.2f", humidity);

        if (temperature > TEMPERATURE_OPEN_LOUVRE && prev_temperature <= TEMPERATURE_OPEN_LOUVRE) {
            write_buf = 0xFF;
            ESP_LOGI(TAG, "Send Stop Fwd (0) to PCF8574");
            ESP_ERROR_CHECK(i2c_master_transmit(pfc8574_out_handle, &write_buf, 1, -1));
            xLastWakeTime = xTaskGetTickCount();
            xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
        
            write_buf = 0xFE;
            ESP_LOGI(TAG, "Send Forward motor enable (0x01) to PCF8574");
            ESP_ERROR_CHECK(i2c_master_transmit(pfc8574_out_handle, &write_buf, 1, -1));  
            
            isOpen = 1;
        }

        if (temperature < TEMPERATURE_CLOSE_LOUVRE && prev_temperature >= TEMPERATURE_CLOSE_LOUVRE) {
            write_buf = 0xFF;
            ESP_LOGI(TAG, "Send Stop (0) to PCF8574");
            ESP_ERROR_CHECK(i2c_master_transmit(pfc8574_out_handle, &write_buf, 1, -1));
            xLastWakeTime = xTaskGetTickCount();
            xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );
            vTaskDelay(pdMS_TO_TICKS(10));
    
            write_buf = 0xF9;
            ESP_LOGI(TAG, "Send Reverse motor direction (0x06) to PCF8574");
            ESP_ERROR_CHECK(i2c_master_transmit(pfc8574_out_handle, &write_buf, 1, -1));
    
            write_buf = 0xF8;
            ESP_LOGI(TAG, "Send Reverse motor enable (0x07) to PCF8574");
            ESP_ERROR_CHECK(i2c_master_transmit(pfc8574_out_handle, &write_buf, 1, -1));

            isOpen = -1;
        }
    
        displayTempHumidity(disp, temperature, humidity);

        send_espnow_data(temperature, humidity, isOpen);

        DELAY_5S;
    }

}
