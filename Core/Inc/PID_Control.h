/*
 * PID_Control.h
 *
 *  Created on: Mar 19, 2025
 *      Author: Step
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include "main.h"

#define TEMP_APPROACH_VALUE	150

#define TEMP_TURBO_LIMIT_DIFF_UST_ARKA	60
#define TEMP_TURBO_LIMIT_DIFF_ALT		48

void PID_Run();
void PID_Setup(void);
void PID_Turbo_Check(void);
void TempSetPoint_Arrive_Check(void);
void manual_pwm_update(void);
void pwmOutProcess(void);
void PID_animationProcess(void);

#endif /* INC_PID_CONTROL_H_ */
