
#ifndef BALANCE_CHASSIS_BSP_BUZZER_H
#define BALANCE_CHASSIS_BSP_BUZZER_H

#include "struct_typedef.h"

void buzzerOn(uint16_t psc, uint16_t pwm);
void buzzerOff(void);
void buzzerBeep(uint8_t times, uint8_t interval);

#endif
