#include "multiplexDisplay.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// From pin in chip to stm32 pin
constexpr uint16_t PM[] = {
    0,  // Dummy as the pins are 1-indexed
    GPIO8,
    GPIO15,
    GPIO14,
    GPIO13,
    GPIO12,
    // Segments start here
    GPIO0,
    GPIO1,
    GPIO2,
    GPIO3,
    GPIO4,
    GPIO5,
    GPIO6,
    GPIO7
};

constexpr uint16_t digits[] = {
    PM[8]  | PM[9]  | PM[11] | PM[12] | PM[13] | PM[7],             // 0
    PM[11] | PM[12],                                                // 1
    PM[9]  | PM[8]  | PM[6]  | PM[12] | PM[13],                     // 2
    PM[9]  | PM[11] | PM[6]  | PM[12] | PM[13],                     // 3
    PM[7]  | PM[6]  | PM[12] | PM[11],                              // 4
    PM[13] | PM[7]  | PM[6]  | PM[11] | PM[9],                      // 5
    PM[13] | PM[7]  | PM[6]  | PM[8]  | PM[9]  | PM[11],            // 6
    PM[13] | PM[12] | PM[11],                                       // 7
    PM[7]  | PM[8]  | PM[9]  | PM[11] | PM[6]  | PM[12] | PM[13],   // 8
    PM[13] | PM[7]  | PM[12] | PM[6]  | PM[11],                     // 9
    0xFF,  // lots on                                               // 10
    0                                                               // Off
};




void MultiplexDisplayTask::setup() {

    // Common (Anode/Cathode)
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12 | GPIO13 | GPIO14 | GPIO15);

    // Segments
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);

    // Set to a default state of all off
    // Common to HIGH
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    // Segment to LOW
    gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);



    // Mapping of GPIO to display pins:
    /*
     1 = PA8
     2 = PB15
     3 = PB14
     4 = PB13
     5 = PB12
     6 = PA0
     7 = PA1
     8 = PA2
     9 = PA3
    10 = PA4
    11 = PA5
    12 = PA6
    13 = PA7
    */

   // Initialise digits & gpio override values
   digitValues[0] = 0;
   digitValues[1] = 0;
   digitValues[2] = 0;
   digitValues[3] = 0;

   // Treat as 1 indexed
   gpioValues[1] = digits[10];
   gpioValues[2] = digits[10];
   gpioValues[3] = digits[11];
   gpioValues[4] = digits[10];
   gpioValues[5] = digits[10];

   digitValues[0] = 0;
   digitValues[1] = 1;
   digitValues[2] = 2;
   digitValues[3] = 3;



}

void MultiplexDisplayTask::loop() {

    static uint16_t timer2 = 0;

    static uint8_t currentCommon = 1;


    // Turn all off
    // Common to HIGH
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    // Segment to LOW
    gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);

    uint16_t gpioPin = gpioValues[currentCommon];
    if (gpioPin) {
        // Don't set anything if there are no segemnts lit up

        // Turn on segments
        gpio_set(GPIOA, gpioPin);

        // Turn on common
        uint32_t gpioPort;
        if (currentCommon == 1){
            gpioPort = GPIOA;
        } else {
            gpioPort = GPIOB;
        }
        gpio_clear(gpioPort, PM[currentCommon]);


        timer2++;
        if (timer2 > 100){
            timer2 = 0;
            // Set GPIO values based on digits
            gpioValues[1] = digits[digitValues[0]];
            gpioValues[2] = digits[digitValues[1]];
            gpioValues[4] = digits[digitValues[2]];
            gpioValues[5] = digits[digitValues[3]];

            
            if (digitValues[3]++ > 8){
                digitValues[3] = 0;
                if (digitValues[2]++ > 8){
                    digitValues[2] = 0;
                    if (digitValues[1]++ > 8) {
                        digitValues[1] = 0;
                        if (digitValues[0]++ > 8){
                            digitValues[0] = 0;
                        }
                    }
                }
            }
            

        }


        // Only need to wait if we need to show a lights for this common
        sleep(1);
    }









    currentCommon++;
    if(currentCommon >= 5+1) {
        currentCommon = 1;
    }

    

    














    /*
    // Turn all off
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    // Turn one common on
    gpio_clear(GPIOA, GPIO8);  // Left most
    for (int num=0;num<=8;num++) {
        // Turn off all
        gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);
        // Turn 1 segment on
        uint16_t gpio = digits[num];
        gpio_set(GPIOA, gpio);
        sleep(100);
    }

    // Turn all off
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    // Turn one common on
    gpio_clear(GPIOB, GPIO15);  // Middle left
    for (int num=0;num<=8;num++) {
        // Turn off all
        gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);
        // Turn 1 segment on
        uint16_t gpio = digits[num];
        gpio_set(GPIOA, gpio);
        sleep(100);
    }

    // Turn all off
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    // Turn one common on
    gpio_clear(GPIOB, GPIO14);  // Middle + top dots
    for (int num=0;num<=8;num++) {
        // Turn off all
        gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);
        // Turn 1 segment on
        uint16_t gpio = digits[num];
        gpio_set(GPIOA, gpio);
        sleep(100);
    }

    // Turn all off
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    // Turn one common on
    gpio_clear(GPIOB, GPIO13);  // Middle right
    for (int num=0;num<=8;num++) {
        // Turn off all
        gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);
        // Turn 1 segment on
        uint16_t gpio = digits[num];
        gpio_set(GPIOA, gpio);
        sleep(100);
    }

    // Turn all off
    gpio_set(GPIOA, GPIO8);
    gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14 | GPIO15);
    // Turn one common on
    gpio_clear(GPIOB, GPIO12);  // Right
    for (int num=0;num<=8;num++) {
        // Turn off all
        gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7);
        // Turn 1 segment on
        uint16_t gpio = digits[num];
        gpio_set(GPIOA, gpio);
        sleep(100);
    }
    */

}
