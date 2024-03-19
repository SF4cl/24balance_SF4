
#include "bsp_can.h"
#include "main.h"
#include "can.h"

#include "app_can.h"

uint8_t can_rx_buf[8];
extern motor_measure_t motor_measure[5];
int left_trim_flag, right_trim_flag;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_rx_buf);
    decodeCanMsg(can_rx_buf, rx_header);
    switch (rx_header.StdId){
        case CAN_TRIM_L_MOTOR_ID:
            if (motor_measure[2].ecd - motor_measure[2].last_ecd > 4000)
                left_trim_flag = 0;
            if (motor_measure[2].ecd - motor_measure[2].last_ecd < -4000)
                left_trim_flag = 1;
            break;
        case CAN_TRIM_R_MOTOR_ID:
            if (motor_measure[3].ecd - motor_measure[3].last_ecd > 4000)
                right_trim_flag = 0;
            if (motor_measure[3].ecd - motor_measure[3].last_ecd < -4000)
                right_trim_flag = 1;
            break;
        default:
            break;
    }
    
    
}

void canFilterInit(void) {
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
