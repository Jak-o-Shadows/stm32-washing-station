#include "commandHandler.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// From pin in chip to stm32 pin


void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len) {
    CommandHandlerTask* mthis = static_cast<CommandHandlerTask*>(tf->userdata);
}




void CommandHandlerTask::setup() {
    tf->userdata = (void*) this;
    //TF_InitStatic(tf, TF_SLAVE);

}

void CommandHandlerTask::loop() {

    // Prepare local buffer to receive bytes to from the stream buffer
    uint8_t localBuffer[1];//TF_USE_SOF_BYTE + TF_ID_BYTES + TF_LEN_BYTES + TF_TYPE_BYTES];
    constexpr TickType_t receiveWaitTicks = 100;  // TODO: Determine this properly

    size_t numBytesReady = xStreamBufferReceive(*uartRxStreamBufferHandle,
                                              &localBuffer,
                                              1,
                                              receiveWaitTicks);
    if (numBytesReady){
        //TF_Accept(tf, &localBuffer[0], numBytesReady);
    }



    // Enable transmit interrupt so it sends back the data.
    //USART_CR1(USART2) |= USART_CR1_TXEIE;
}
