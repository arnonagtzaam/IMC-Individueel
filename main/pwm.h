#include "board.h"
#include "esp_log.h"

#define ADDR PCA9685_ADDR_BASE
#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 18
#define SCL_GPIO 23
#endif
#define FREQUENCY 255
#define PWM_VALUE 1000

#define RED 2
#define GREEN 0
#define BLUE 1

#ifdef __cplusplus
extern "C" 
{
    #endif

    void pca9685_initialize();
    void setup_input_keys();
    void initialize_audio_chip();
    void set_pwm_value(int channel, uint16_t val);
    void disco_task(void *params);
    void fade_animation();
    void set_color(int red, int green, int blue);
    void _set_color();
    void fade_task(void *params);

    #ifdef __cplusplus
}
#endif