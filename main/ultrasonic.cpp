#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/portmacro.h>
#include <esp_log.h>
#include <esp_timer.h>

#define TAG "Ultrasonic"

#define ECHO_PIN GPIO_NUM_34
#define TRIG_PIN GPIO_NUM_32

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 5000
#define ROUNDTRIP 58
#define MAX_DISTANCE 500
#define SURFACE_SENSIVITY 5

#define time() esp_timer_get_time()
#define timeout_expired(start, len) ((uint32_t)(time() - (start)) >= (len))

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

esp_timer *timer;

static bool isSurfaceUnderSensor() {
    portENTER_CRITICAL(&mux);

    // Ping: Low for 2..4 us, then high 10 us
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(TRIGGER_HIGH_DELAY);
    gpio_set_level(TRIG_PIN, 0);

    if (gpio_get_level(ECHO_PIN)) {
        portEXIT_CRITICAL(&mux);
        ESP_LOGE(TAG, "ECHO is still 1");
        return false;
    }

    uint32_t start = time();
    while (!gpio_get_level(ECHO_PIN)) {
        if (timeout_expired(start, PING_TIMEOUT)) {
            portEXIT_CRITICAL(&mux);
            ESP_LOGE(TAG, "ECHO pings timed out");
            return false;
        }
    }

    // got echo, measuring
    uint32_t echo_start = time();
    uint32_t time = echo_start;
    uint32_t meas_timeout = echo_start + MAX_DISTANCE * ROUNDTRIP;
    while (gpio_get_level(ECHO_PIN)) {
        time = time();
        if (timeout_expired(echo_start, meas_timeout)) {
            portEXIT_CRITICAL(&mux);
            ESP_LOGE(TAG, "ECHO timed out");
            return false;
        }
    }
    portEXIT_CRITICAL(&mux);
    return ((time - echo_start) / ROUNDTRIP) < SURFACE_SENSIVITY;
}

static void sensor_main() {
    esp_timer_init();

    gpio_config_t config = {
            .pin_bit_mask = 1ULL << ECHO_PIN,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);

    config = {
            .pin_bit_mask = 1ULL << TRIG_PIN,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);
}