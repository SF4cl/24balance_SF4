
#ifndef BALANCE_CHASSIS_APP_CAN_H
#define BALANCE_CHASSIS_APP_CAN_H

#define DEDICATED_CAN_HANDLE         (hcan2)
#define SHARED_CAN_HANDLE            (hcan1)

// NOTE: chassis motors are all on CAN2
#define CAN_CHASSIS_6020_ALL_ID_1234 (0x1ff)
#define CAN_CHASSIS_6020_ALL_ID_567  (0x2ff)
#define CAN_GIMBAL_YAW_MOTOR_ID      (0x205)     // 6020 #1 205@CAN2
#define CAN_TRIM_R_MOTOR_ID          (0x207)     // 6020 #2 207@CAN1
#define CAN_TRIM_L_MOTOR_ID          (0x206)     // 6020 #3 206@CAN1
#define CAN_CHASSIS_3508_ALL_ID_1234 (0x200)
#define CAN_CHASSIS_3508_ALL_ID_5678 (0x1ff)
#define CAN_R_MOTOR_ID               (0x201)     // 3508 #1 201@CAN2
#define CAN_L_MOTOR_ID               (0x202)     // 3508 #2 202@CAN2

#define CAN_BOARD_COMM_ID            (0x2ff)     // used for inter-board communication

#include <stdint.h>

#include "can.h"

#include "app_remote.h"

// motor measure data structure
typedef struct {
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

// RC control data structure
typedef struct
{
    int16_t ch[5];
    char s[2];
} RC_ctrl_t;

void decodeCanMsg(uint8_t *buf, CAN_RxHeaderTypeDef header);
void canCmdChassis6020(int16_t trim_l_throttle, int16_t trim_r_throttle);
void canCmdChassis3508(int32_t l_throttle, int32_t r_throttle);
void canCmdChassis(int16_t trim_l, int16_t trim_r, int16_t l, int16_t r);

#endif
