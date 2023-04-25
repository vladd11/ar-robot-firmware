/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string>
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <esp_websocket_client.h>
#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <charconv>
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_oneshot.h>

#include "lwip/err.h"
#include "ultrasonic.h"

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define IP_FORMAT "%lu.%lu.%lu.%lu"
#define IP_ARGS(ip) ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, (ip >> 24) & 0xff

#define SSID "SSAU_310"
#define PASSWORD CONFIG_ESP_WIFI_PASSWORD
#define CHANNEL CONFIG_ESP_WIFI_CHANNEL
#define MAX_CONNECTIONS CONFIG_ESP_MAX_STA_CONN

static const char *TAG = "wifi softAP";

const gpio_num_t GPIO_IN[] = {GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17}; // IN1/2/3/4/ENA/ENB

#define CAMERA_SERVO_GPIO             GPIO_NUM_19
#define MOVEMENT_SERVO_GPIO             GPIO_NUM_21
#define HAND_SERVO_GPIO             GPIO_NUM_25
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD        20000

// SERVO
// MOVEMENT_BORDER_LEFT  590
// MOVEMENT_BORDER_RIGHT 2325

#define SPEED_LOWER 1350
#define SPEED_HIGHER 1600

#define MOVEMENT_FORWARD 1650
#define MOVEMENT_LEFT 1000
#define MOVEMENT_RIGHT 2310

static esp_websocket_client *clients[MAX_CONNECTIONS];
static adc_oneshot_unit_handle_t adc_handle;
static TaskHandle_t xElevatorHandle;
static TaskHandle_t xHandHandle;
static int elevatorLevel;

static int curr = 0;

#define CAMERA_SERVO 0
#define MOVEMENT_SERVO 1
mcpwm_cmpr_handle_t comparators[3];

static QueueHandle_t gpio_evt_queue;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        auto *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG, "station "
                MACSTR
                " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        auto *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG, "station "
                MACSTR
                " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void ws_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
    auto *event = (esp_websocket_event_data_t *) event_data;
    if (event_id == WEBSOCKET_EVENT_DATA) {
        const char *ptr = event->data_ptr + event->payload_offset;
        std::string str(ptr, event->payload_len);
        str.erase(std::find_if(str.rbegin(), str.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), str.end());

        if (str.empty()) return;

        ESP_LOGI(TAG, "%s %c", str.c_str(), str[0]);
        int value;
        switch (str[0]) {
            case 'C':
                if (str.length() < 2) return;

                // Camera
                std::from_chars(&str[1], &str[str.length()], value);
                ESP_LOGI(TAG, "CAMERA: %i", value);

                // 1470 1515
                mcpwm_comparator_set_compare_value(comparators[CAMERA_SERVO], value);
                vTaskDelay(200);
                mcpwm_comparator_set_compare_value(comparators[CAMERA_SERVO], 0);
                break;
            case 'S':
                if (str.length() < 2) return;

                // Camera
                std::from_chars(&str[1], &str[str.length()], value);
                ESP_LOGI(TAG, "MOVEMENT: %i", value);

                mcpwm_comparator_set_compare_value(comparators[MOVEMENT_SERVO], value);
                break;
            case 'A':
                if (str.length() < 2) return;

                // 1600 / 1150
                std::from_chars(&str[1], &str[str.length()], value);
                ESP_LOGI(TAG, "HAND: %i", value);

                mcpwm_comparator_set_compare_value(comparators[2], value);
                break;
            case 'E':
                // Motors
                if (str.length() != 5) return;

                for (int i = 1; i < 5; i++) {
                    ESP_LOGI(TAG, "Pin: %i state %i", i - 1, str[i] - '0');
                    gpio_set_level(GPIO_IN[i - 1], str[i] - '0');
                }
                break;
            case 'H':
                std::from_chars(&str[1], &str[str.length()], elevatorLevel);
                vTaskResume(xElevatorHandle);
                break;
            case 'P':
                vTaskResume(xHandHandle);
                break;
            case 'L':
                vTaskSuspend(xHandHandle);
                break;
        }
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "%li", event_id);
    if (event_id == IP_EVENT_AP_STAIPASSIGNED) {
        auto *event = (ip_event_ap_staipassigned_t *) event_data;
        uint32_t ip = event->ip.addr;

        char url[28];
        sprintf(url, "ws://" IP_FORMAT":8000", IP_ARGS(ip));
        ESP_LOGI(TAG, "%s", url);
        esp_websocket_client_config_t config = {
                .uri = url,
                .reconnect_timeout_ms = 3000,
                .network_timeout_ms = 10000,
        };
        esp_websocket_client *client = esp_websocket_client_init(&config);
        esp_err_t err = esp_websocket_client_start(client);

        esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, ws_event_handler, nullptr);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Connected to " IP_FORMAT, IP_ARGS(ip));
            clients[curr] = client;
            curr++;
        } else {
            ESP_LOGE(TAG, "Error perform http request %s", esp_err_to_name(err));
        }
    }
}

void wifi_init_softap() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &ip_event_handler,
                                                        nullptr,
                                                        nullptr));

    wifi_config_t wifi_config = {
            .ap = {
                    .ssid = SSID,
                    .password = PASSWORD,
                    .ssid_len = static_cast<uint8_t>(strlen(SSID)),
                    .channel = CHANNEL,
                    .authmode = WIFI_AUTH_WPA_WPA2_PSK,
                    .max_connection = MAX_CONNECTIONS,
                    .pmf_cfg = {
                            .required = false,
                    },
            },
    };
    if (strlen(PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             SSID, PASSWORD, CHANNEL);
}

void servo_main(int group, int id, gpio_num_t gpio, int def) {
    mcpwm_timer_handle_t timer = nullptr;
    mcpwm_timer_config_t timer_config = {
            .group_id = group,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = SERVO_TIMEBASE_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    mcpwm_oper_handle_t oper = nullptr;
    mcpwm_operator_config_t operator_config = {
            .group_id = group, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparator_config = {
            .flags={
                    .update_cmp_on_tez = true
            },
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparators[id]));

    mcpwm_gen_handle_t generator = nullptr;
    mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = gpio,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[id], def));

    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator,
                                                               MCPWM_GEN_TIMER_EVENT_ACTION(
                                                                       MCPWM_TIMER_DIRECTION_UP,
                                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                                       MCPWM_GEN_ACTION_HIGH),
                                                               MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generator,
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(
                                                                         MCPWM_TIMER_DIRECTION_UP, comparators[id],
                                                                         MCPWM_GEN_ACTION_LOW),
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

static void elevator_task(void *arg) { // 165 / 800
    int val;
    for (;;) {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &val);
        if (abs(val - elevatorLevel) < 7) {
            mcpwm_comparator_set_compare_value(comparators[CAMERA_SERVO], 0);
            ESP_LOGI(TAG, "SUSPENDED");
            vTaskSuspend(nullptr);
        } else {
            ESP_LOGI(TAG, "%i %i", val, elevatorLevel);
            if (val < elevatorLevel) {
                mcpwm_comparator_set_compare_value(comparators[CAMERA_SERVO], SPEED_LOWER);
            } else {
                mcpwm_comparator_set_compare_value(comparators[CAMERA_SERVO], SPEED_HIGHER);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    auto gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static bool edge = false;

static void hand_sensor(void *arg) {
    for (;;) {
        if (isSurfaceUnderSensor()) {
            for (int i = 0; i < curr; ++i) {
                esp_websocket_client_send_text(clients[i], "Y", sizeof "Y", portMAX_DELAY);
            }
        } else {
            for (int i = 0; i < curr; ++i) {
                esp_websocket_client_send_text(clients[i], "N", sizeof "N", portMAX_DELAY);
            }
        }
        vTaskDelay(100);
    }
}


extern "C" void app_main() {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uint64_t mask = 0;
    for (gpio_num_t pin: GPIO_IN) {
        mask |= 1ULL << pin;
    }
    gpio_config_t config = {
            .pin_bit_mask = mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_NUM_34, gpio_isr_handler, (void *) GPIO_NUM_35);

    adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    xTaskCreate(elevator_task, "elevator", 4096, nullptr, tskIDLE_PRIORITY, &xElevatorHandle);
    vTaskSuspend(xElevatorHandle);

    sensor_main();
    xTaskCreate(hand_sensor, "hand", 4096, nullptr, tskIDLE_PRIORITY, &xHandHandle);
    vTaskSuspend(xHandHandle);

    servo_main(0, 0, CAMERA_SERVO_GPIO, 0);
    servo_main(1, 1, MOVEMENT_SERVO_GPIO, MOVEMENT_FORWARD);
    servo_main(1, 2, HAND_SERVO_GPIO, 1600);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
}
