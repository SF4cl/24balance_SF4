
#include "bsp_led.h"

#include "main.h"


extern TIM_HandleTypeDef htim5;


void ledSetColor(uint32_t argb_value) {
    static uint8_t alpha;
    static uint16_t red, green, blue;

    alpha = (argb_value & 0xFF000000) >> 24;
    red = ((argb_value & 0x00FF0000) >> 16) * alpha;
    green = ((argb_value & 0x0000FF00) >> 8) * alpha;
    blue = ((argb_value & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
