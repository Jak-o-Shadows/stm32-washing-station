#include "turntable.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


void TurntableTask::setup() {
}

void TurntableTask::loop() {
    sleep(500000000);
}
