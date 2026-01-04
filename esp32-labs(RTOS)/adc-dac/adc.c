#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"
#include <inttypes.h>


#define SAMPLE_RATE_MS 100   // 10 Hz sampling
#define ADC_CHANNEL ADC1_CHANNEL_5  // GPIO6
#define ATTEN ADC_ATTEN_DB_11 // 0-3V
#define ADC_UNIT ADC_UNIT_1 //ADC1
#define DEFAULT_VREF 1100 //reference voltage for calibration

static const char *TAG = "ADC_SAMPLE"; //for ESP_LOG messages

QueueHandle_t adc_queue; // global handle for FreeRTOS queue, 
                        // transports data between tasks
// Calibration
esp_adc_cal_characteristics_t adc_chars;

// Sampling Task
void adc_sampling_task(void *arg) {
    uint32_t adc_reading = 0;

    while (1) {
        adc_reading = adc1_get_raw(ADC_CHANNEL); //read ADC values
        xQueueSend(adc_queue, &adc_reading, portMAX_DELAY); //store in queue
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_RATE_MS)); //delay so sample rate is as described
    }
}
// Data Sending Task (UART)
void uart_send_task(void *arg) {
    uint32_t sample;

    while (1) {
        if (xQueueReceive(adc_queue, &sample, portMAX_DELAY)) { //blocks until new sample arrives
            uint32_t voltage = esp_adc_cal_raw_to_voltage(sample, &adc_chars); //converting to volts for readability
            printf("ADC Raw: %" PRIu32 ", Voltage: %" PRIu32 "mV\n", sample, voltage); //sends data to serial monitor via UART
        }
    }
}

void app_main(void) {
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12); //12bit resolution
    adc1_config_channel_atten(ADC_CHANNEL, ATTEN); //11db to channel 5
    esp_adc_cal_characterize(ADC_UNIT, ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars); //set ADC characteristics

    adc_queue = xQueueCreate(10, sizeof(uint32_t)); //create queue for 10 samples

    // Create tasks
    xTaskCreate(adc_sampling_task, "adc_sampling_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_send_task, "uart_send_task", 2048, NULL, 5, NULL);
}
