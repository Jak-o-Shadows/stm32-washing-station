#include "maintask.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


void myenableGpioRcc(uint32_t gpioPort) {
    rcc_periph_clken rccPeriph;
    switch (gpioPort){
        case GPIOA:
            rccPeriph = RCC_GPIOA;
            break;
        case GPIOB:
            rccPeriph = RCC_GPIOB;
            break;
        case GPIOC:
            rccPeriph = RCC_GPIOC;
            break;
        case GPIOD:
            rccPeriph = RCC_GPIOD;
            break;
    }
    rcc_periph_clock_enable(rccPeriph);
}


void MainTask::setup() {
	uvLedPort = GPIOA;
    uvLedPin = GPIO9;
    
	visLedPort = GPIOA;
    visLedPin = GPIO10;

    myenableGpioRcc(uvLedPort);
    myenableGpioRcc(visLedPort);

    gpio_set_mode(uvLedPort, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, uvLedPin);
    gpio_set_mode(visLedPort, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, visLedPin);

    gpio_clear(uvLedPort, uvLedPin);
    gpio_clear(visLedPort, visLedPin);

}

void MainTask::loop() {

    // Turn both LED's on
    gpio_set(uvLedPort, uvLedPin);
    gpio_set(visLedPort, visLedPin);

    for (int second=0; second<60*5; second++){  // Looping cause not show of when the int overflows
        sleep(1000);
    }
    gpio_clear(uvLedPort, uvLedPin);
    gpio_clear(visLedPort, visLedPin);

    vTaskSuspend(xHandle);  // Just power cycle to get it to turn back on
}
