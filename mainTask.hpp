#ifndef TASK_MAINTASK_H_
#define TASK_MAINTASK_H_

#include "RtosTask.hpp"

class MainTask: public RtosTask {
private:
	void setup() override;
	void loop() override;

    uint32_t uvLedPin;
	uint32_t visLedPin;
	uint32_t uvLedPort;
	uint32_t visLedPort;
};

#endif