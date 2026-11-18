#include "fan.h"
#include "firebase.h"

#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* Manual button GPIOs */
#define INPUT_OFF       GPIO_NUM_18
#define INPUT_LOW       GPIO_NUM_19
#define INPUT_NORMAL    GPIO_NUM_21
#define INPUT_HIGH      GPIO_NUM_22

/* Fan output GPIOs */
#define OUTPUT_OFF      GPIO_NUM_23
#define OUTPUT_LOW      GPIO_NUM_25
#define OUTPUT_NORMAL   GPIO_NUM_26
#define OUTPUT_HIGH     GPIO_NUM_27

#define FAN_COMMAND_QUEUE_LENGTH 5
#define BUTTON_DEBOUNCE_MS      1000

/* Task notification bits */
#define BUTTON_OFF_BIT       (1UL << FAN_OFF)
#define BUTTON_LOW_BIT       (1UL << FAN_LOW)
#define BUTTON_NORMAL_BIT    (1UL << FAN_NORMAL)
#define BUTTON_HIGH_BIT      (1UL << FAN_HIGH)

#define REMOTE_COMMAND_BIT   (1UL << 4)

#define BUTTON_EVENT_MASK ( \
    BUTTON_OFF_BIT |       \
    BUTTON_LOW_BIT |       \
    BUTTON_NORMAL_BIT |    \
    BUTTON_HIGH_BIT        \
)

static const char *TAG = "Fan";

static const gpio_num_t input_pins[FAN_SPEED_COUNT] = {
    [FAN_OFF]    = INPUT_OFF,
    [FAN_LOW]    = INPUT_LOW,
    [FAN_NORMAL] = INPUT_NORMAL,
    [FAN_HIGH]   = INPUT_HIGH
};

static const gpio_num_t output_pins[FAN_SPEED_COUNT] = {
    [FAN_OFF]    = OUTPUT_OFF,
    [FAN_LOW]    = OUTPUT_LOW,
    [FAN_NORMAL] = OUTPUT_NORMAL,
    [FAN_HIGH]   = OUTPUT_HIGH
};

static QueueHandle_t fan_command_queue = NULL;
static TaskHandle_t fan_task_handle = NULL;

/* fan_task owns the applied fan state. */
static fan_speed_t current_speed = FAN_OFF;
static portMUX_TYPE fan_state_mux = portMUX_INITIALIZER_UNLOCKED;

static void fan_set_speed(fan_speed_t speed)
{
    if (speed < FAN_OFF || speed >= FAN_SPEED_COUNT) {
        return;
    }

    portENTER_CRITICAL(&fan_state_mux);
    bool changed = (speed != current_speed);
    portEXIT_CRITICAL(&fan_state_mux);

    if (!changed) {
        return;
    }

    /* Break before make: disable all outputs first. */
    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        gpio_set_level(output_pins[i], 0);
    }

    /* Enable the selected output. */
    gpio_set_level(output_pins[speed], 1);

    portENTER_CRITICAL(&fan_state_mux);
    current_speed = speed;
    portEXIT_CRITICAL(&fan_state_mux);

    /* Firebase reporting remains asynchronous. */
    firebase_state_changed(speed);
}

fan_speed_t fan_get_speed(void)
{
    fan_speed_t speed;

    portENTER_CRITICAL(&fan_state_mux);
    speed = current_speed;
    portEXIT_CRITICAL(&fan_state_mux);

    return speed;
}

static void fan_button_isr(void *arg)
{
    fan_speed_t speed = (fan_speed_t)(uintptr_t)arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    xTaskNotifyFromISR(fan_task_handle, (1UL << speed), eSetBits, &higher_priority_task_woken);

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void fan_buttons_enable_interrupts(void)
{
    esp_err_t err = gpio_install_isr_service(0);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        return;
    }

    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        /* Active-low: falling edge means button pressed. */
        err = gpio_set_intr_type(input_pins[i], GPIO_INTR_NEGEDGE);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure GPIO %d", (int)input_pins[i]);
            continue;
        }

        err = gpio_isr_handler_add(input_pins[i], fan_button_isr, (void *)(uintptr_t)i);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register ISR for GPIO %d", (int)input_pins[i]);
            continue;
        }
        ESP_LOGI(TAG, "Interrupt enabled on GPIO %d", (int)input_pins[i]);
    }
}

static bool fan_process_button_events(uint32_t button_events)
{
    /* Register the initial press immediately, without requiring the button to remain held. */
    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        if ((button_events & (1UL << i)) == 0) {
            continue;
        }

        ESP_LOGI(TAG, "Manual fan selection: %d", i);
        fan_set_speed((fan_speed_t)i);
    }

    /* Ignore further manual button interrupts during the debounce lockout.
       Remote commands remain queued/notified and are processed afterward. */
    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        gpio_intr_disable(input_pins[i]);
    }

    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));

    /* Discard any stale button notifications from before/during lockout,
       without clearing a pending remote-command notification. */
    uint32_t ignored_button_events = 0;
    xTaskNotifyWait(0, BUTTON_EVENT_MASK, &ignored_button_events, 0);

    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        gpio_intr_enable(input_pins[i]);
    }

    return false;
}

static void fan_process_remote_commands(void)
{
    fan_command_t command;

    while (xQueueReceive(fan_command_queue, &command, 0) == pdPASS) {
        if (command.speed < FAN_OFF ||
            command.speed >= FAN_SPEED_COUNT) {

            ESP_LOGW(TAG, "Ignoring invalid fan command");
            continue;
        }

        ESP_LOGI(TAG,
                 "Applying remote command: id=%lld, speed=%d",
                 (long long)command.command_id,
                 (int)command.speed);

        fan_set_speed(command.speed);
    }
}

esp_err_t fan_command_submit(const fan_command_t *command)
{
    if (command == NULL ||
        command->speed < FAN_OFF ||
        command->speed >= FAN_SPEED_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (fan_command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(fan_command_queue, command, 0) != pdPASS) {
        ESP_LOGW(TAG,
                 "Command queue full; command %lld was not queued",
                 (long long)command->command_id);

        return ESP_ERR_TIMEOUT;
    }

    /* Wake fan_task to process the remote command. */
    if (fan_task_handle != NULL) {
        xTaskNotify(
            fan_task_handle,
            REMOTE_COMMAND_BIT,
            eSetBits
        );
    }

    return ESP_OK;
}

void fan_init(void)
{
    fan_command_queue = xQueueCreate(
        FAN_COMMAND_QUEUE_LENGTH,
        sizeof(fan_command_t)
    );
    if (fan_command_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    gpio_config_t input_config = {
        .pin_bit_mask =
            (1ULL << INPUT_OFF) |
            (1ULL << INPUT_LOW) |
            (1ULL << INPUT_NORMAL) |
            (1ULL << INPUT_HIGH),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config_t output_config = {
        .pin_bit_mask =
            (1ULL << OUTPUT_OFF) |
            (1ULL << OUTPUT_LOW) |
            (1ULL << OUTPUT_NORMAL) |
            (1ULL << OUTPUT_HIGH),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&input_config));
    ESP_ERROR_CHECK(gpio_config(&output_config));

    /* Start with all outputs disabled. */
    for (int i = 0; i < FAN_SPEED_COUNT; i++) {
        ESP_ERROR_CHECK(gpio_set_level(output_pins[i], 0));
    }

    /* Default fan state is OFF. */
    ESP_ERROR_CHECK(gpio_set_level(output_pins[FAN_OFF], 1));

    portENTER_CRITICAL(&fan_state_mux);
    current_speed = FAN_OFF;
    portEXIT_CRITICAL(&fan_state_mux);
}

void fan_task(void *pvParameters)
{
    fan_task_handle = xTaskGetCurrentTaskHandle();
    fan_buttons_enable_interrupts();
    ESP_LOGI(TAG, "Fan task started");

    while (true) {
        fan_process_remote_commands();

        uint32_t notification = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notification, portMAX_DELAY);
        if (notification & REMOTE_COMMAND_BIT) {
            fan_process_remote_commands();
        }

        uint32_t button_events = notification & BUTTON_EVENT_MASK;
        if (button_events != 0) {
            bool remote_command_received =
                fan_process_button_events(button_events);

            if (remote_command_received) {
                fan_process_remote_commands();
            }
        }
    }
}
