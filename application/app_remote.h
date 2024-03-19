
#ifndef BALANCE_CHASSIS_APP_REMOTE_H
#define BALANCE_CHASSIS_APP_REMOTE_H

#include "struct_typedef.h"


#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_LEFT_SWITCH_CH       ((uint8_t)1)
#define RC_RIGHT_SWITCH_CH      ((uint8_t)0)
#define RC_SW_UP                ((uint8_t)1)
#define RC_SW_MID               ((uint8_t)3)
#define RC_SW_DOWN              ((uint8_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)


#endif
