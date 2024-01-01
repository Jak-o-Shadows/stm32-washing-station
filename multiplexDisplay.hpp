#ifndef TASK_MULTIPLEXDISPLAYTASK_H_
#define TASK_MULTIPLEXDISPLAYTASK_H_

#include "RtosTask.hpp"

class MultiplexDisplayTask: public RtosTask {
private:
	void setup() override;
	void loop() override;

	uint16_t gpioValues[5+1];  // make it 1 indexed
	uint8_t digitValues[4];  // The values of the 4 digits (right to left)

};

#endif