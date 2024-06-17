#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#define GPIO_OUTPUT_IO_0 17
#define GPIO_OUTPUT_IO_1 18
#define GPIO_OUTPUT_IO_2 19
#define GPIO_OUTPUT_PIN_SEL                                                    \
  ((1ULL << GPIO_OUTPUT_IO_0) | (1ULL << GPIO_OUTPUT_IO_1) |                   \
   (1ULL << GPIO_OUTPUT_IO_2))
enum State { RED_LIGHT, GREEN_LIGHT, YELLOW_LIGHT };

static bool trafic_light(gptimer_handle_t timer,
                         const gptimer_alarm_event_data_t *edata,
                         void *user_ctx);

static void change_light(enum State state);

void app_main(void) {

  // LED

  gpio_config_t led = {.mode = GPIO_MODE_OUTPUT,
                       .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
                       .pull_down_en = 0,
                       .pull_up_en = 0,
                       .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&led);

  // TIMER
  QueueHandle_t queue = xQueueCreate(1, sizeof(int));
  enum State state = RED_LIGHT; //

  // Enviar o dado para a fila
  xQueueSend(queue, &state, portMAX_DELAY);

  gptimer_handle_t gptimer = NULL;
  gptimer_config_t timer_config = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
                                   .direction = GPTIMER_COUNT_UP,
                                   .resolution_hz = 1 * 1000 * 1000, // 1us
                                   .intr_priority = 1};

  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  gptimer_alarm_config_t alarm_config = {.alarm_count = 1000000,
                                         .reload_count = 0,
                                         .flags.auto_reload_on_alarm = true};

  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  // Estado inicial RED
  gptimer_event_callbacks_t cbs = {
      .on_alarm = trafic_light,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));

  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  ESP_ERROR_CHECK(gptimer_start(gptimer));
}

static void change_light(enum State state) {
  if (state == RED_LIGHT) {
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_2, 0);

  } else if (state == GREEN_LIGHT) {
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_1, 1);
    gpio_set_level(GPIO_OUTPUT_IO_2, 0);

  } else {
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_2, 1);
  }
}

static void update_state(enum State *state) {
  if (*state == RED_LIGHT) {
    *state = GREEN_LIGHT;
  } else if (*state == GREEN_LIGHT) {
    *state = YELLOW_LIGHT;
  } else {
    *state = RED_LIGHT;
  }
}

static bool trafic_light(gptimer_handle_t timer,
                         const gptimer_alarm_event_data_t *edata,
                         void *user_ctx) {

  enum State state;

  xQueueReceive(user_ctx, &state, portMAX_DELAY);

  change_light(state);
  update_state(&state);
  xQueueSend(user_ctx, &state, portMAX_DELAY);

  BaseType_t high_task_awoken = pdFALSE;

  gptimer_alarm_config_t alarm_config = {
      .alarm_count = edata->alarm_value + 1000000,
  };
  gptimer_set_alarm_action(timer, &alarm_config);

  return high_task_awoken == pdTRUE;
}
