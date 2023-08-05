#include "OledTask.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


void OledTask::setup() {

    const uint32_t i2c = I2C2; //i2c2

    // Set up peripherals
    i2cMaster_setup(i2c);
    i2cDma_setup();

    // Set up what is being drawn logic
    uint8_t rotateX = 0;


    // Initialise device properties
    SSD1306_t dev = {
        dev.state = DEAD,
        dev.address = 0b0111100,
        dev.mode = PAGE,
        dev.i2c = I2C2
    };

    // Initialise multiple buffer & tracking variables
    segment = 0;  // of the overall screen
    segmentIdx = 0;  // of the segments

    init(&dev);

    // Send some data

    // Delay
    for (int i = 0; i < 10000; i++)
    {
        __asm__("nop");
    }

    // Clear the display
    OLED_address(&dev, 0, 0);
    for (int pxByte = 0; pxByte < 128 * 32 / 8; pxByte++)
    {
        uint8_t bytes[] = {
            0x40,
            0};
        i2cMaster_send(I2C2, dev.address, bytes, 2);
    }

    // Configure scrolling
    setupScroll(&dev, false);
    scrollState(&dev, false);

    // kick it all off
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    uint8_t bytes[] = {0xFF};
    i2cSendBytesDMA(dev.address, bytes, 1);

}


void OledTask::loop() {

    sleep(msRotate);

    // Do the update
    rotateX = rotateX - 5;
    // Kick off update
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    uint8_t bytes[] = {0xFF};
    i2cSendBytesDMA(dev.address, bytes, 1);


}


Status OledTask::i2cSendBytesDMA(uint8_t addr, uint8_t data[], uint8_t numData)
{

    dma_disable_channel(DMA1, DMA_CHANNEL4);

    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)data);

    dma_set_number_of_data(DMA1, DMA_CHANNEL4, numData);

    // Start the transaction
    uint32_t reg32 __attribute__((unused));

    //send start
    i2c_send_start(I2C2);

    //wait for the switch to master mode.
    while (!((I2C_SR1(I2C2) & I2C_SR1_SB) &
             (I2C_SR2(I2C2) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
        ;

    //send address
    i2c_send_7bit_address(I2C2, addr, I2C_WRITE);
    //check for the ack
    while (!(I2C_SR1(I2C2) & I2C_SR1_ADDR))
        ;
    //must read SR2 to clear it
    reg32 = I2C_SR2(I2C2);

    // Send the data
    i2c_enable_dma(I2C2);

    dma_enable_channel(DMA1, DMA_CHANNEL4);

    uint16_t maxCheck = 65000;
    while (maxCheck--)
    {
        if ((I2C_SR1(I2C2) & I2C_SR1_BTF))
        {
            break;
        }
    }

    // Finish the transaction
    i2c_send_stop(I2C2);
    for (int i = 0; i < 200; i++)
    {
        __asm__("nop");
    }

    // Disable DMA
    i2c_disable_dma(I2C2);
    //dma_disable_channel(DMA1, DMA_CHANNEL4);

    return STATUSok;
}


void OledTask::i2cDma_setup(void)
{

    // Setup DMA with the i2c
    //  @100 kHz, one byte takes 0.08 ms. => 1 pixel takes 0.01 ms
    //  Hence the whole 128x32 screen takes a minimum of 4096*0.01 ms
    //      = 40.96 ms.
    // Ignoring a couple of things, this makes a full update rate of
    //  just 24 Hz
    //
    // If using a line-by line paradigm, with a 32 pixel line, each line
    //  takes 0.32 ms to transfer. This is ONLY an update rate of 3125 Hz
    //      => the line by line paradigm probably works pretty damn easy
    //
    // i2c is slow...

    // Use DMA to transfer
    // I2C2 is DMA1 Channel 4 (TX)
    // Setup DMA
    dma_channel_reset(DMA1, DMA_CHANNEL4);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&I2C2_DR);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);

    dma_disable_channel(DMA1, DMA_CHANNEL4);
}