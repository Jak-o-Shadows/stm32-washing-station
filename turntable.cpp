#include "turntable.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


void enableGpioRcc(uint32_t gpioPort) {
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


void TurntableTask::setup() {
    pulsePort = GPIOB;
    pulsePin = GPIO10;
    directionPort = GPIOB;
    directionPin = GPIO11;

    enableGpioRcc(pulsePort);
    enableGpioRcc(directionPort);

    gpio_set_mode(pulsePort, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pulsePin);
    gpio_set_mode(pulsePort, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, directionPin);

    gpio_clear(pulsePort, pulsePin);
    gpio_clear(directionPort, directionPin);

    sleepTime_ms = 10;

}

void TurntableTask::loop() {
    gpio_toggle(pulsePort, pulsePin);
    sleep(sleepTime_ms);
}
