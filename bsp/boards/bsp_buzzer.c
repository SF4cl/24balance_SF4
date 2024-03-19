
#include "bsp_buzzer.h"

#include "cmsis_os.h"

#include "main.h"


extern TIM_HandleTypeDef htim4;


void buzzerOn(uint16_t psc, uint16_t pwm) {
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}

void buzzerOff(void) {
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzerBeep(uint8_t times, uint8_t interval) {
    for (uint8_t i = 0; i < times; i++) {
        buzzerOn(1, 30000);
        osDelay(interval);
        buzzerOff();
        osDelay(interval);
    }
}
