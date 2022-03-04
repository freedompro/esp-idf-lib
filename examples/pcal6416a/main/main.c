#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <pcal6416a.h>
#include <driver/gpio.h>
#include <string.h>

// A0, A1, A2 pins are grounded

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define INT_GPIO 16
#define RESET_GPIO 10
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define INT_GPIO 34
#define RESET_GPIO 32
#define SDA_GPIO 9
#define SCL_GPIO 10
#endif

static void IRAM_ATTR gpio_isr_handler_task(void *arg)
{
    printf("Interrupt!\n");

    uint16_t port_read;
    uint16_t int_port_read;
    i2c_dev_t *dev = (i2c_dev_t *)arg;

    pcal6416a_port_get_interupt_status(dev, &int_port_read);
    printf("int portRead, %d\n", int_port_read);

    pcal6416a_port_read(dev, &port_read);
    printf("portRead, %d\n", (port_read & int_port_read) ? 1 : 0);

    vTaskDelete(NULL);
}

static void intr_handler(void *arg)
{
    xTaskCreate(gpio_isr_handler_task, "gpio_isr_handler_task", 6 * 512, arg, 0, NULL);
}

void test(void *pvParameters)
{
    i2c_dev_t *dev = (i2c_dev_t *)malloc(sizeof(i2c_dev_t));

    memset(dev, 0, sizeof(i2c_dev_t));

    // setup interupt
    gpio_pad_select_gpio(INT_GPIO);
    gpio_set_direction(INT_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(INT_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT_GPIO, intr_handler, (void *)dev);

    gpio_pad_select_gpio(RESET_GPIO);
    gpio_set_direction(RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_GPIO, 0);

    // vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set_level(RESET_GPIO, 1);

    // vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(pcal6416a_init_desc(dev, 0, pcal6416a_I2C_ADDR_BASE, SDA_GPIO, SCL_GPIO));

    // Setup P03, P05, P06, P07, P10, P12, P13, P14, P15, P16, P17 as input, other as output
    u_int16_t mode = 65000; // 0b1111110111101000
    ESP_ERROR_CHECK(pcal6416a_port_set_mode(dev, mode));
    ESP_ERROR_CHECK(pcal6416a_port_set_interrupt_mask(dev, ~mode));
    ESP_ERROR_CHECK(pcal6416a_port_set_input_latch(dev, mode));
    ESP_ERROR_CHECK(pcal6416a_port_set_rev_polarity(dev, mode));

    // blink on P04
    bool on = true;

    while (1)
    {
        ESP_ERROR_CHECK(pcal6416a_set_level(dev, 4, on));
        on = !on;

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}