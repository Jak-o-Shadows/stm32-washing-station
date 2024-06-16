#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/i2c.h>
#include <libopencm3/stm32/dma.h>

#include <libopencm3/cm3/nvic.h>

#include <stdint.h>
#include <string.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#include "status.hpp"

#include "taskBlink.hpp"
//#include "OledTask.hpp"
#include "turnTable.hpp"
#include "multiplexDisplay.hpp"
#include "commandHandler.hpp"

//#include "devices/gy521/gy521.hpp"


HeartbeatTask heartbeat;
TurntableTask turntable;
//OledTask oled;
MultiplexDisplayTask display;
CommandHandlerTask commandHandler;


// Set up the stream buffers for the Uart ISR <-> task
#define UART_STREAM_BUFFER_SIZE_BYTES 1024
static uint8_t uartRxStreamBufferMemory[ UART_STREAM_BUFFER_SIZE_BYTES + 1 ];
static uint8_t uartTxStreamBufferMemory[ UART_STREAM_BUFFER_SIZE_BYTES + 1 ];
StaticStreamBuffer_t uartRxStreamBufferStruct;
StaticStreamBuffer_t uartTxStreamBufferStruct;
StreamBufferHandle_t uartRxStreamBuffer;
StreamBufferHandle_t uartTxStreamBuffer;




void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_AFIO);

    rcc_periph_clock_enable(RCC_GPIOB);
    //I2C
    rcc_periph_clock_enable(RCC_I2C2);
    rcc_periph_clock_enable(RCC_DMA1);
}

void usart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);

    gpio_set_mode(RCC_GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2); //USART 2 TX is A2
    gpio_set_mode(RCC_GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3);                   //USART 2 RX is A3

    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    //enable interrupt rx
    USART_CR1(USART1) |= USART_CR1_RXNEIE;

    usart_enable(USART1);
}

static void nvic_setup(void)
{
    nvic_set_priority(NVIC_USART1_IRQ, 2);
    nvic_enable_irq(NVIC_USART1_IRQ);

    // For SSD1036
//    nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);  // Probably wrong, I changed the i2C pins
//    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

//    nvic_set_priority(NVIC_TIM2_IRQ, 2);
//    nvic_enable_irq(NVIC_TIM2_IRQ);
}

static void gpio_setup(void)
{
    //setup i2c pins
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                  GPIO8 | GPIO9); //B8 =SCL, B9=SDA
}


/////////////////////////////////////////////////////
////////////// Main Loop   //////////////////////////
/////////////////////////////////////////////////////

int main(void)
{

    // Properly initiailse the streambuffers
    uartRxStreamBuffer = xStreamBufferCreateStatic( UART_STREAM_BUFFER_SIZE_BYTES,
                                               1,
                                               uartRxStreamBufferMemory,
                                               &uartRxStreamBufferStruct );
    uartTxStreamBuffer = xStreamBufferCreateStatic( UART_STREAM_BUFFER_SIZE_BYTES,
                                               1,
                                               uartTxStreamBufferMemory,
                                               &uartTxStreamBufferStruct );
    commandHandler.uartRxStreamBufferHandle = &uartRxStreamBuffer;
    commandHandler.uartTxStreamBufferHandle = &uartTxStreamBuffer;




    clock_setup();
    gpio_setup();
    usart_setup();
    nvic_setup();

    /*
    const uint32_t i2c = I2C2; //i2c2
    i2cMaster_setup(i2c);

    GY521_t gy521_dev = {
        .address = 0x68,
        .i2c = I2C2,
    };
    int16_t accels[3];
    int16_t gyros[3];
    
    Status sts;
    sts = gy521_init(&gy521_dev);

    //while(1){
    {
        for(int i=0;i<50000;i++){
            __asm__("nop");
        };
        sts = gy521_read(&gy521_dev, accels, gyros);

    };
    */

    
    heartbeat.start("heartbeat", configMINIMAL_STACK_SIZE, 0);  // 0 is lowest priority
    //oled.start("oled", configMINIMAL_STACK_SIZE, 1);
    turntable.start("turntable", configMINIMAL_STACK_SIZE, 100);
    display.start("display", configMINIMAL_STACK_SIZE, 1);
    commandHandler.start("commandHandler", configMINIMAL_STACK_SIZE, 2);

    vTaskStartScheduler();
    for(;;);
    
    return 0;

}

/////////////////////////////////////////////////////
////////////// Comms Stuff //////////////////////////
/////////////////////////////////////////////////////

// Interrupt Functions

void usart1_isr(void)
{
    static uint8_t data = 'A';
    static uint8_t reply = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* Initialised to pdFALSE. */

    /* Check if we were called because of RXNE. */
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_RXNE) != 0))
    {
        // Receieve the data, using the MiniSSC protocol
        //	This protocol has a header byte (0xFF), followed
        //	by a number (0->254) followed by a number (0-254)
        data = usart_recv(USART1);
        size_t numBytesSent = xStreamBufferSendFromISR(uartRxStreamBuffer,
                                                    &data,
                                                    1,
                                                    &xHigherPriorityTaskWoken);
        if (numBytesSent != 1){
            // Could not sent the data? Maybe it's full?
        }
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_TXE) != 0))
    {

        // Indicate that we are sending out data.
        // gpio_toggle(GPIOA, GPIO7);

        // Put data into the transmit register.
        uint8_t toSendBuf[10];
        size_t numBytesToSend = xStreamBufferReceiveFromISR(uartTxStreamBuffer,
                                                            &toSendBuf[0],
                                                            10,
                                                            &xHigherPriorityTaskWoken);
        for (int idx; idx<numBytesToSend; idx++) {
            usart_send(USART1, toSendBuf[idx]);
        }

        // Disable the TXE interrupt as we don't need it anymore.
        USART_CR1(USART1) &= ~USART_CR1_TXEIE;
    }
}


/*
void dma1_channel4_isr(void)
{
    // I2C Transmit DMA channel

    // First clear flag
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF);
    }

    if (oled.segment == 0)
    {
        OLED_address(&oled.dev, 0, 0);
    }
    oled.segment++;
    if ((oled.segment*oled.display.segment_width*oled.display.segment_height) >= 
        (oled.display.display_width * oled.display.display_height) )
    {
        oled.segment = 0;
        // Disable TCIF interrupt flag
        dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    }
    else
    {
        // Must re-enable TCIF interrupt to start it again
        dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    }

    // Send the initial buffer
    oled.i2cSendBytesDMA(oled.dev.address, oled.display.segments[oled.segmentIdx].data.bytes, oled.display.segments[oled.segmentIdx].bytes_len);

    // Then swap buffer to prepare the next one.
    oled.segmentIdx++;
    if (oled.segmentIdx >= oled.display.num_segments)
    {
        oled.segmentIdx = 0;
    }

    // Then prepare next buffer to send

    // First clear it
    uint8_t fill = 0;
    if (oled.segmentIdx==0){
        fill = 0;
    } else {
        fill = 0xFF;
    }
    memset(oled.display.segments[oled.segmentIdx].data.components.pixelBuffer, fill, oled.display.segments[oled.segmentIdx].bytes_len-1);

    // Draw a line across X near the bottom
    uint8_t offset;
    for (uint8_t row = 0; row < oled.display.display_height; row++)
    {
        offset = oled.segment*4;
        offset = offset % oled.display.display_height;
        pixelInvert(&oled.display.segments[oled.segmentIdx], oled.display.getSegmentXCoord(oled.segment), oled.display.getSegmentYCoord(oled.segment), oled.display.segment_width - offset, row);
    }

    // Draw some characters
     char word[] = "Jak_o_Shadows";
    uint8_t wordLength = 13;
    uint8_t start = oled.rotateX + oled.display.display_height/2;
    words(&oled.display.segments[oled.segmentIdx], oled.display.getSegmentXCoord(oled.segment), oled.display.getSegmentYCoord(oled.segment), start, 0, word, wordLength);

    gpio_toggle(GPIOC, GPIO14);

}
*/

