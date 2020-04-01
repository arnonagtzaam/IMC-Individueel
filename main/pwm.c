#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "http_stream.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"

#include "periph_button.h"
#include "input_key_service.h"
#include "periph_touch.h"
#include "esp_peripherals.h"
#include "periph_wifi.h"
#include "board.h"
#include <pca9685.h>
#include <pwm.h>

static const char *TAG = "PWM";

periph_service_handle_t input_ser;
esp_periph_set_handle_t set;
audio_board_handle_t board_handle;
i2c_dev_t device;

int r = 0;
int g = 0;
int b = 0;

typedef struct 
{
    bool red, green, blue;
    int disco;
} RGB_BOOLS_STRUCT;

RGB_BOOLS_STRUCT rgb_bools = 
{ 
    false, false, false,
    0 
};

// Initialize the PWM module
void pca9685_initialize()
{
    memset(&device, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(pca9685_init_desc(&device, ADDR, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&device));

    ESP_ERROR_CHECK(pca9685_restart(&device));

    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&device, FREQUENCY));

    initialize_audio_chip();
    setup_input_keys();
}

// Set value to PWM module
void set_pwm_value(int channel, uint16_t val)
{
    ESP_LOGI(TAG, "Set ch0 to %d", val);
    if (pca9685_set_pwm_value(&device, channel, val) != ESP_OK)
        ESP_LOGE(TAG, "Could not set PWM value to ch0");
}

// Input key callback
static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) 
    {
        switch ((int)evt->data) 
        {
            case INPUT_KEY_USER_ID_PLAY:
			{
                ESP_LOGI(TAG, "RED");
                if(!rgb_bools.red)
                {
                    set_pwm_value(RED, PWM_VALUE);
                } 
                else 
                {
                    set_pwm_value(RED, 0);
                }
                rgb_bools.red = !rgb_bools.red;
                break;
			}
            case INPUT_KEY_USER_ID_SET:
            {
                ESP_LOGI(TAG, "GREEN");
                if(!rgb_bools.green)
                {
                    set_pwm_value(GREEN, PWM_VALUE);
                } 
                else 
                {
                    set_pwm_value(GREEN, 0);
                }
                rgb_bools.green = !rgb_bools.green;
                break;
            }
            case INPUT_KEY_USER_ID_VOLDOWN:
			{
                ESP_LOGI(TAG, "BLUE");
                if(!rgb_bools.blue)
                {
                    set_pwm_value(BLUE, PWM_VALUE);
                } 
                else 
                {
                    set_pwm_value(BLUE, 0);
                }
                rgb_bools.blue = !rgb_bools.blue;
                break;
			}
            case INPUT_KEY_USER_ID_VOLUP:
			{
                ESP_LOGI(TAG, "DISCO");
                if(rgb_bools.disco == 0)
                {
                    xTaskCreate(&disco_task, "disco_task", 1024 * 2, NULL, 5, NULL);
                    rgb_bools.disco++;
                } 
                else if (rgb_bools.disco == 1)
                {
                    xTaskCreate(&fade_task, "fade_task", 1024 * 2, NULL, 5, NULL);
                    rgb_bools.disco++;
                }
                else 
                {
                    rgb_bools.disco++;
                } 
                break;
			}
        }
    }
    return ESP_OK;
}

// Initialize audio chip
void initialize_audio_chip()
{
    ESP_LOGI(TAG, "Initialize peripherals management and start codec chip");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    set = esp_periph_set_init(&periph_cfg);
    audio_board_key_init(set);
    board_handle = audio_board_init();
}

// Initialize input keys
void setup_input_keys()
{
    ESP_LOGI(TAG, "Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_ser = input_key_service_create(set);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);
}

// Task that creates disco effect on RGB LED
void disco_task(void *params)
{
    while(rgb_bools.disco == 1)
    {
        set_pwm_value(0, PWM_VALUE);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_pwm_value(0,0);
        set_pwm_value(1, PWM_VALUE);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_pwm_value(1,0);
        set_pwm_value(2, PWM_VALUE);
        vTaskDelay(100 / portTICK_RATE_MS);
        set_pwm_value(2,0);
    }
    vTaskDelete(NULL);
}

// Task that creates fade effect on RGB LED
void fade_task(void *params)
{
    while(rgb_bools.disco == 2)
    {
        fade_animation();
    }
    set_pwm_value(RED, 0);
    set_pwm_value(GREEN, 0);
    set_pwm_value(BLUE, 0);
    rgb_bools.disco = 0;
    vTaskDelete(NULL);
}

// Sets colors to be faded
void fade_animation() { 
  set_color(PWM_VALUE, 0, 0);
  set_color(0, PWM_VALUE, 0);
  set_color(0, 0, PWM_VALUE);
}

// Main fading method
void set_color(int red, int green, int blue) {
  while ( (rgb_bools.disco == 2) && (r != red || g != green || b != blue)) {
    if ( r < red ) r += 5;
    if ( r > red ) r -= 5;

    if ( g < green ) g += 5;
    if ( g > green ) g -= 5;

    if ( b < blue ) b += 5;
    if ( b > blue ) b -= 5;

    _set_color();
    vTaskDelay(10/ portTICK_RATE_MS);
  }
}

// Setting all PWM channels at the same time
void _set_color() {
  set_pwm_value(RED, r);
  set_pwm_value(GREEN, g);
  set_pwm_value(BLUE, b); 
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    pca9685_initialize();
}