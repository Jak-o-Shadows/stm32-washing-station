#ifndef TASK_COMMANDHANDLERTASK_H_
#define TASK_COMMANDHANDLERTASK_H_

#include "RtosTask.hpp"
#include "stream_buffer.h"

#include "TinyFrame.h"
#include "TF_Config.h"

class CommandHandlerTask: public RtosTask {
public:
StreamBufferHandle_t* uartTxStreamBufferHandle;
StreamBufferHandle_t* uartRxStreamBufferHandle;
private:
	void setup() override;
	void loop() override;
	TinyFrame* tf;


};

#endif