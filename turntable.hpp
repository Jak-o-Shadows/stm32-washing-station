#ifndef TASK_TURNTABLETASK_H_
#define TASK_TURNTABLETASK_H_

#include "RtosTask.hpp"

class TurntableTask: public RtosTask {
private:
	void setup() override;
	void loop() override;

	uint32_t pulsePort;
	uint32_t pulsePin;
	uint32_t directionPort;
	uint32_t directionPin;

	int sleepTime_ms;

};

#endif