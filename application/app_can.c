
#include "app_can.h"

#include <string.h>


/***
 * @brief motor measure data structure
 * @note  [0] -> left motor       3508 0x202@CAN1 ID=2
 * @note  [1] -> right motor      3508 0x201@CAN1 ID=1
 * @note  [2] -> left trim motor  6020 0x206@CAN1 ID=2
 * @note  [3] -> right trim motor 6020 0x207@CAN1 ID=3
 * @note  [4] -> gimbal yaw motor 6020 0x205@CAN2 ID=1
 */
motor_measure_t motor_measure[5];

RC_ctrl_t rc_ctrl;


static int8_t getMotorIndexByID(uint16_t id) {
    switch (id) {
        case CAN_L_MOTOR_ID:
            return 0;
        case CAN_R_MOTOR_ID:
            return 1;
        case CAN_TRIM_L_MOTOR_ID:
            return 2;
        case CAN_TRIM_R_MOTOR_ID:
            return 3;
        case CAN_GIMBAL_YAW_MOTOR_ID:
            return 4;
        default:
            return -1;
    }
}


/***
 * @note inter-board communication packet structure
 * @note big endian
 * @note [0] [1] -> left stick L/R: u16
 * @note [2] [3] -> left stick U/D: u16
 * @note [6]     -> left switch: u8
 */
void decodeCanMsg(uint8_t *buf, CAN_RxHeaderTypeDef header) {
    switch (header.StdId) {
        case CAN_L_MOTOR_ID:
        case CAN_R_MOTOR_ID:
        case CAN_TRIM_L_MOTOR_ID:
        case CAN_TRIM_R_MOTOR_ID:
        case CAN_GIMBAL_YAW_MOTOR_ID: {
            int8_t motor_index = getMotorIndexByID(header.StdId);
            motor_measure[motor_index].last_ecd = motor_measure[motor_index].ecd;
            motor_measure[motor_index].ecd = (uint16_t)((buf[0] << 8) | buf[1]);
            motor_measure[motor_index].speed_rpm = (int16_t)((buf[2] << 8) | buf[3]);
            motor_measure[motor_index].given_current = (int16_t)((buf[4] << 8) | buf[5]);
            motor_measure[motor_index].temperate = buf[6];
            break;
        }
        case CAN_BOARD_COMM_ID: {
            rc_ctrl.ch[0] = (int16_t)((buf[0] << 8) | buf[1]);
            rc_ctrl.ch[1] = (int16_t)((buf[2] << 8) | buf[3]);
            rc_ctrl.s[1] = buf[6];
            break;
        }
        default:
            break;

    }
}


uint8_t can_send_data_buffer[8];
CAN_TxHeaderTypeDef chassis_can_tx_header = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 0x08,
};


void canCmdChassis6020(int16_t trim_l_throttle, int16_t trim_r_throttle) {
    uint32_t send_mail_box;
    chassis_can_tx_header.StdId = CAN_CHASSIS_6020_ALL_ID_1234;

    memset(can_send_data_buffer, 0, sizeof(can_send_data_buffer));

    can_send_data_buffer[2] = trim_l_throttle >> 8;
    can_send_data_buffer[3] = trim_l_throttle;
    can_send_data_buffer[4] = trim_r_throttle >> 8;
    can_send_data_buffer[5] = trim_r_throttle;
    HAL_CAN_AddTxMessage(&DEDICATED_CAN_HANDLE, &chassis_can_tx_header, can_send_data_buffer, &send_mail_box);
}


void canCmdChassis3508(int32_t l_throttle, int32_t r_throttle) {
    uint32_t send_mail_box;
    chassis_can_tx_header.StdId = CAN_CHASSIS_3508_ALL_ID_1234;

    memset(can_send_data_buffer, 0, sizeof(can_send_data_buffer));

    can_send_data_buffer[0] = l_throttle >> 8;
    can_send_data_buffer[1] = l_throttle;
    can_send_data_buffer[2] = r_throttle >> 8;
    can_send_data_buffer[3] = r_throttle;
    HAL_CAN_AddTxMessage(&DEDICATED_CAN_HANDLE, &chassis_can_tx_header, can_send_data_buffer, &send_mail_box);
}


void canCmdChassis(int16_t trim_l, int16_t trim_r, int16_t l, int16_t r) {
    canCmdChassis6020(trim_l, trim_r);
    canCmdChassis3508(l, r);
}
