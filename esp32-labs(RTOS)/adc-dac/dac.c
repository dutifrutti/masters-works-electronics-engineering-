/**
 *  DDS-driven LED “breathing” demo
 *  --------------------------------
 *  – LED on GPIO 2 fades with a 1 Hz sine envelope
 *  – PWM carrier (200 kHz) is far above sight, so the LED looks steady
 *  – Sample/update rate = 1 kHz  →  1 ms timing granularity
 */

 #include <math.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_log.h"
 #include "driver/ledc.h"
 
 #define LED_GPIO            6           // connect LED (+ resistor) here
 #define PWM_TIMER_HZ        20000     // hardware limit
 #define PWM_RES_BITS        11          // up to 8191 duty steps
 #define SAMPLE_RATE_HZ      1000        // 1 kSa/s → 1 ms per sample
 #define OUTPUT_FREQ_HZ      1.0         // **visible 1-Hz breathing**
 #define LUT_BITS            8
 #define LUT_SIZE            (1U << LUT_BITS)
 #define PWM_DUTY_MAX        ((1U << PWM_RES_BITS) - 1)
 
 static const char *TAG = "LED_BREATH";
 static uint16_t sin_quarter[LUT_SIZE];
 
 /* ---------- LUT generation (¼ sine) ---------------- */
 static void init_quarter_lut(void)
 {
     for (uint32_t i = 0; i < LUT_SIZE; ++i) {
         double s = sin((double)i / LUT_SIZE * M_PI_2);   // 0 … +1
         sin_quarter[i] = (uint16_t)(s * PWM_DUTY_MAX + 0.5);
     }
 }
 
 /* ---------- fast symmetry lookup ------------------- */
 static inline uint16_t IRAM_ATTR dds_sample(uint32_t phase)
 {
     uint32_t q   = phase >> 30;                                     // quadrant
     uint32_t idx = (phase >> (30 - LUT_BITS)) & (LUT_SIZE - 1);     // 8 bits
 
     uint16_t v = (q & 1) ? sin_quarter[LUT_SIZE - 1 - idx]
                          : sin_quarter[idx];
     if (q & 2) v = PWM_DUTY_MAX - v;                                // invert Q2,Q3
     return v;
 }
 
 /* ---------- LEDC PWM initialisation --------------- */
 static void ledc_init(void)
 {
     ledc_timer_config_t t = {
         .speed_mode      = LEDC_LOW_SPEED_MODE,
         .timer_num       = LEDC_TIMER_0,
         .duty_resolution = PWM_RES_BITS,
         .freq_hz         = PWM_TIMER_HZ,
         .clk_cfg         = LEDC_AUTO_CLK,
     };
     ledc_timer_config(&t);
 
     ledc_channel_config_t c = {
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .channel    = LEDC_CHANNEL_0,
         .timer_sel  = LEDC_TIMER_0,
         .gpio_num   = LED_GPIO,
         .duty       = 0,
         .hpoint     = 0,
     };
     ledc_channel_config(&c);
 }
 
 /* ---------- FreeRTOS task: 1 kSa/s DDS ------------ */
 static void IRAM_ATTR dds_task(void *arg)
 {
     const uint32_t phase_inc =
         (uint32_t)(( (double)(1ULL << 32) * OUTPUT_FREQ_HZ) / SAMPLE_RATE_HZ);
 
     uint32_t   phase        = 0;
     TickType_t tick_period  = pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ);
     TickType_t next_wakeup  = xTaskGetTickCount();
 
     while (true) {
         uint16_t duty = dds_sample(phase);
 
         ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
         ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
 
         phase += phase_inc;
         vTaskDelayUntil(&next_wakeup, tick_period);
     }
 }
 
 /* ---------- app entry ----------------------------- */
 void app_main(void)
 {
     ESP_LOGI(TAG, "Starting LED breathing demo (%g Hz)...", OUTPUT_FREQ_HZ);
     init_quarter_lut();
     ledc_init();
 
     xTaskCreatePinnedToCore(dds_task, "dds_led",
                             2048, NULL, configMAX_PRIORITIES - 1,
                             NULL, 0);
 }
 