#ifndef TASK_OLED_HPP_
#define TASK_OLED_HPP_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/i2c.h>
#include <libopencm3/stm32/dma.h>

#include <libopencm3/cm3/nvic.h>

#include "RtosTask.hpp"

// Library Includes
#include "macros.h"
#include "status.hpp"
#include "devices/ssd1306/ssd1306.hpp"
#include "protocol/drawing/drawing.hpp"
#include "periph/i2c/i2cMaster.hpp"


class OledTask: public RtosTask {
public:
    // Working variables
    SSD1306_t dev;
    uint8_t segment;
    uint8_t segmentIdx;
    Display<SCREEN_WIDTH, SCREEN_HEIGHT, WIDTH, HEIGHT, 2> display;

    // Temp - stuff to cause it to change
    uint8_t rotateX;


    Status i2cSendBytesDMA(uint8_t addr, uint8_t data[], uint8_t numData);
private:
	void setup() override;
	void loop() override;

    void i2cDma_setup();

    // TEMP - drawing function
    const uint16_t msRotate = 1000;

};

#endif