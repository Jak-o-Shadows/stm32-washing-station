#include "commandHandler.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "commands.hpp"



// TinyFrame required
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len) {
    CommandHandlerTask* mthis = static_cast<CommandHandlerTask*>(tf->userdata);

    constexpr TickType_t receiveWaitTicks = 100;  // TODO: Determine this properly

    size_t numBytesSend = xStreamBufferSend(*mthis->uartTxStreamBufferHandle,
                                            buff,
                                            len,
                                            receiveWaitTicks);
    if (numBytesSend != len) {
        // Log something
    }
}

// Command callbacks

TF_Result listenerTimeoutFunction(TinyFrame *tf) {
    // TODO: Log
    return TF_CLOSE;
}

TF_Result demoIdListener(TinyFrame *tf, TF_Msg *msg) {
    return TF_CLOSE;
}

TF_Result demoGenericListener(TinyFrame *tf, TF_Msg *msg) {
    return TF_STAY;  // 
}

TF_Result demoTypeListener(TinyFrame *tf, TF_Msg *msg) {
    return TF_CLOSE;
}




// Main RTOS task

void CommandHandlerTask::setup() {
    tf->userdata = (void*) this;
    TF_InitStatic(tf, TF_SLAVE);

    // Generic Callback listeners
    TF_AddGenericListener(tf, demoGenericListener);

    // ID listeners
    TF_Msg msg;  // ID listeners use a message to provide the filter data
    msg.frame_id = static_cast<TF_ID>(MsgId::blah2);  // TODO: No idea why the static_cast is needed
    TF_AddIdListener(tf, &msg, demoIdListener, listenerTimeoutFunction, 10000);  // No idea what kind of value the timeouts should be

    // Type listener
    //TF_AddTypeListener(tf, MsgType::c, demoTypeListener);


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
        TF_Accept(tf, &localBuffer[0], numBytesReady);
    }

    TF_Tick(tf);


    // Enable transmit interrupt so it sends back the data.
    USART_CR1(USART2) |= USART_CR1_TXEIE;
}
