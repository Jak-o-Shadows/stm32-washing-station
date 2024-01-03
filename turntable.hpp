#ifndef TASK_TURNTABLETASK_H_
#define TASK_TURNTABLETASK_H_

#include "RtosTask.hpp"

class TurntableTask: public RtosTask {
private:
	void setup() override;
	void loop() override;
};

#endif