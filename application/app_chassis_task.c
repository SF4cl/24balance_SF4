
#include "app_chassis_task.h"

#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

#include "pid.h"
#include "app_can.h"
#include "BMI088driver.h"
#include "bsp_buzzer.h"
#include "app_debug_plotter_task.h"


extern motor_measure_t motor_measure[5];

//         -        +
// [0]  CW <- yaw   -> CCW
// [1]   L <- roll  -> R 
// [2]   L <- pitch -> H
extern bmi088_real_data_t bmi088_real_data;
extern fp32 INS_angle[3];
extern RC_ctrl_t rc_ctrl;
extern int left_trim_flag, right_trim_flag; 

pid_t chassis_pid[3];                // 0: linear velocity loop, 1: upright loop, 2: yaw position loop
                                     // 3: left trim position loop, 4: right trim position loop

pid_t left_trim_speed_pid, right_trim_speed_pid; // 0: left trim position loop, 1: right trim position loop
pid_t left_trim_current_pid, right_trim_current_pid; // 0: left trim current loop, 1: right trim current loop
pid_t trim_pos_pid, upright_trim_pid;

fp32 chassis_linear_velocity,
     chassis_rotation_velocity,
     rpm_left_wheel,
     rpm_right_wheel,
     output_left_wheel,
     output_right_wheel,
     left_trim_ecd,
     right_trim_ecd,
     left_trim_speed,
     right_trim_speed,
     left_trim_current,
     right_trim_current,
     trim_pos,
     left_trim_pos,
     right_trim_pos;

uint16_t pid_init_ramp = 0;

fp32 test_speed_pid_out;

// NOTE don't use rc_lpf[0](which is for linear speed) which will cause chassis to shake
rc_lpf_t rc_lpf[2] = {{0.01f, 0.0f}, {0.1f, 0.0f}};


static void allPIDInit() {
    // linear speed loop
    PID_init(chassis_pid, 
             PID_POSITION,
             (fp32[3]){CHASSIS_LINEAR_SPEED_PID_KP, CHASSIS_LINEAR_SPEED_PID_KI, CHASSIS_LINEAR_SPEED_PID_KD},
             CHASSIS_LINEAR_SPEED_PID_MAX_OUT, CHASSIS_LINEAR_SPEED_PID_MAX_IOUT);
    // upright loop
    PID_init(chassis_pid + 1,
             PID_POSITION,
             (fp32[3]){CHASSIS_UPRIGHT_PID_KP, CHASSIS_UPRIGHT_PID_KI, CHASSIS_UPRIGHT_PID_KD},
             CHASSIS_UPRIGHT_PID_MAX_OUT, CHASSIS_UPRIGHT_PID_MAX_IOUT);
    // yaw position loop
    PID_init(chassis_pid + 2, 
             PID_POSITION,
             (fp32[3]){CHASSIS_YAW_PID_KP, CHASSIS_YAW_PID_KI, CHASSIS_YAW_PID_KD},
             CHASSIS_YAW_PID_MAX_OUT, CHASSIS_YAW_PID_MAX_IOUT);
    // left trim position loop
   PID_init(&left_trim_speed_pid,
            PID_POSITION,
            (fp32[3]){CHASSIS_LEFT_TRIM_SPEED_PID_KP, CHASSIS_LEFT_TRIM_SPEED_PID_KI, CHASSIS_LEFT_TRIM_SPEED_PID_KD},
            CHASSIS_LEFT_TRIM_SPEED_PID_MAX_OUT, CHASSIS_LEFT_TRIM_SPEED_PID_MAX_IOUT);
    // right trim position loop
    PID_init(&right_trim_speed_pid,
            PID_POSITION,
            (fp32[3]){CHASSIS_RIGHT_TRIM_SPEED_PID_KP, CHASSIS_RIGHT_TRIM_SPEED_PID_KI, CHASSIS_RIGHT_TRIM_SPEED_PID_KD},
            CHASSIS_RIGHT_TRIM_SPEED_PID_MAX_OUT, CHASSIS_RIGHT_TRIM_SPEED_PID_MAX_IOUT);
    // left trim current loop
    PID_init(&left_trim_current_pid,
            PID_POSITION,
            (fp32[3]){CHASSIS_LEFT_TRIM_CURRENT_PID_KP, CHASSIS_LEFT_TRIM_CURRENT_PID_KI, CHASSIS_LEFT_TRIM_CURRENT_PID_KD},
            CHASSIS_LEFT_TRIM_CURRENT_PID_MAX_OUT, CHASSIS_LEFT_TRIM_CURRENT_PID_MAX_IOUT);
    // right trim current loop
    PID_init(&right_trim_current_pid,
            PID_POSITION,
            (fp32[3]){CHASSIS_RIGHT_TRIM_CURRENT_PID_KP, CHASSIS_RIGHT_TRIM_CURRENT_PID_KI, CHASSIS_RIGHT_TRIM_CURRENT_PID_KD},
            CHASSIS_RIGHT_TRIM_CURRENT_PID_MAX_OUT, CHASSIS_RIGHT_TRIM_CURRENT_PID_MAX_IOUT);
    PID_init(&trim_pos_pid,
            PID_POSITION,
            (fp32[3]){CHASSIS_TRIM_POSITION_PID_KP, CHASSIS_TRIM_POSITION_PID_KI, CHASSIS_TRIM_POSITION_PID_KD},
            CHASSIS_TRIM_POSITION_PID_MAX_OUT, CHASSIS_TRIM_POSITION_PID_MAX_IOUT);
}


static inline fp32 limitOutput(fp32 input, fp32 max, fp32 min) {
    if (input > max) {
        return max;
    } else if (input < min) {
        return min;
    } else {
        return input;
    }
}


static inline float lpfCalc(rc_lpf_t *rc_lpf, float val) {
    rc_lpf->lVal = ((float)val * rc_lpf->k + rc_lpf->lVal * (1 - rc_lpf->k));
    return rc_lpf->lVal;
}


/***
 * @note 0 <= x <= 1
 * @note -1 <= a <= 1
 */
static inline fp32 curveFunctionStd(fp32 x, fp32 a) {
    if (x > 0.12f) {
        return (a * x * x + (1 - a) * x) * MOTOR_3508_SCALING_FACTOR + 0.12921328f;
    } else {
        return (a * x * x + (1 - a) * x);
    }
}

static inline fp32 curveFunction(fp32 x, fp32 a, fp32 max, fp32 min) {
    return (x >= 0 ? 1 : -1) * (curveFunctionStd((fabs(x) - min) / (max - min), a) * (max - min) + min);
}


/***
 * @brief emergency loop
 * @note to recover from this failsafe loop, left switch must be set in a sequence
 * @note recover sequence
 * @note left switch: down -> middle -> up -> middle -> up
 */
static void emergencyLoop() {
    static const uint8_t manual_recover_sequence[5] = {RC_SW_DOWN, RC_SW_MID, RC_SW_UP, RC_SW_MID, RC_SW_UP};
    static uint8_t switch_input_buf[5] = {0};
    switch_input_buf[4] = rc_ctrl.s[RC_LEFT_SWITCH_CH];
    
    while (1) {
        osDelay(1);
        canCmdChassis(0, 0, 0, 0);

        if (memcmp(switch_input_buf, manual_recover_sequence, sizeof(manual_recover_sequence)) == 0) {
            // manual recover success
            buzzerBeep(1, 100);
            return;
        }

        if (rc_ctrl.s[RC_LEFT_SWITCH_CH] != switch_input_buf[4]) {
            // shift the history to the left
            memmove(switch_input_buf, switch_input_buf + 1, 4 * sizeof(uint8_t));
            // add the new state to the end of the history
            switch_input_buf[4] = rc_ctrl.s[RC_LEFT_SWITCH_CH];
        }
    }
}


void chassisTask(void const *pvParameter) {
    osDelay(9000);  // wait for IMU data to stabilize(this will take about 11 seconds)
                    // 9sec + pid ramp in time(5sec) should be fine
    allPIDInit();
    plotterAddVariable(&INS_angle[0], VARIABLE_TYPE_FP32);
    plotterAddVariable(&INS_angle[1], VARIABLE_TYPE_FP32);
    plotterAddVariable(&INS_angle[2], VARIABLE_TYPE_FP32);
    //plotterAddVariable(&INS_angle[1], VARIABLE_TYPE_FP32);
    //plotterAddVariable(&INS_angle[0], VARIABLE_TYPE_FP32);


    while (1) {
        osDelay(1);
        
        // pause if switch is in middle
        if (switch_is_mid(rc_ctrl.s[RC_LEFT_SWITCH_CH])) {
            PID_clear(&chassis_pid[0]);
            PID_clear(&chassis_pid[1]);
            PID_clear(&chassis_pid[2]);
            PID_clear(&left_trim_speed_pid);
            PID_clear(&right_trim_speed_pid);
            PID_clear(&left_trim_current_pid);
            PID_clear(&right_trim_current_pid);
            continue;
        }

        // suspend main loop if chassis falled down
        if (fabs(INS_angle[2] * 1000) > FALL_ANGLE_THRESHOLD) {
            emergencyLoop();
        }

        // pid ramp in on startup
        if (pid_init_ramp < PID_RAMP_IN_TIME) {
            ++pid_init_ramp;
            chassis_pid[0].Kp = CHASSIS_LINEAR_SPEED_PID_KP * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[0].Ki = CHASSIS_LINEAR_SPEED_PID_KI * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[0].Kd = CHASSIS_LINEAR_SPEED_PID_KD * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[1].Kp = CHASSIS_UPRIGHT_PID_KP * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[1].Ki = CHASSIS_UPRIGHT_PID_KI * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[1].Kd = CHASSIS_UPRIGHT_PID_KD * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[2].Kp = CHASSIS_YAW_PID_KP * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[2].Ki = CHASSIS_YAW_PID_KI * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[2].Kd = CHASSIS_YAW_PID_KD * pid_init_ramp / PID_RAMP_IN_TIME;
        }

#if LEFT_WHEEL_REV
        rpm_left_wheel = -motor_measure[0].speed_rpm;
#else
        rpm_left_wheel = motor_measure[0].speed_rpm;
#endif
#if RIGHT_WHEEL_REV
        rpm_right_wheel = -motor_measure[1].speed_rpm;
#else
        rpm_right_wheel = motor_measure[1].speed_rpm;
#endif

        chassis_linear_velocity = (rpm_left_wheel + rpm_right_wheel) * 0.5f;

        chassis_rotation_velocity = bmi088_real_data.gyro[1] * 1000;

        left_trim_ecd = motor_measure[2].ecd + left_trim_flag * 8191;
        left_trim_ecd = (left_trim_ecd - 7950) * (660 - (-660)) / (16100 - 7950) - 660;

        right_trim_ecd = motor_measure[3].ecd + right_trim_flag * 8191;
        right_trim_ecd = (right_trim_ecd - 8550) * ((-660) - 660) / (350 - 8550) + 660;
        
        //left_trim_ecd = limitOutput(left_trim_ecd, 8000.0f, 500.0f);

        //right_trim_ecd = limitOutput(right_trim_ecd, 8000.0f, 500.0f);

        // low pass filter calculations
        // bypassed, will cause chassis to shake
        chassis_linear_velocity = lpfCalc(&rc_lpf[0], chassis_linear_velocity);
        chassis_rotation_velocity = lpfCalc(&rc_lpf[1], chassis_rotation_velocity);

        // pid calculations
        PID_calc(&chassis_pid[0], chassis_linear_velocity / 1000, -rc_ctrl.ch[1] * MOVE_INPUT_MULTIPLIER);
        PID_calc(&chassis_pid[1], INS_angle[2] * 1000, chassis_pid[0].out);
        PID_calc(&chassis_pid[2], chassis_rotation_velocity, -rc_ctrl.ch[0] * 2);
        output_left_wheel = chassis_pid[1].out + chassis_pid[2].out;
        output_right_wheel = chassis_pid[1].out - chassis_pid[2].out;

        // post processing
        output_left_wheel = limitOutput(output_left_wheel, MAX_3508_MOTOR_CAN_CURRENT, MIN_3508_MOTOR_CAN_CURRENT);
        output_right_wheel = limitOutput(output_right_wheel, MAX_3508_MOTOR_CAN_CURRENT, MIN_3508_MOTOR_CAN_CURRENT);
        output_left_wheel = curveFunction(output_left_wheel, CHASSIS_PID_OUTPUT_CURVE_FACTOR, MAX_3508_MOTOR_CAN_CURRENT, 0);
        output_right_wheel = curveFunction(output_right_wheel, CHASSIS_PID_OUTPUT_CURVE_FACTOR, MAX_3508_MOTOR_CAN_CURRENT, 0);

        // trim calculations
        PID_calc(&trim_pos_pid, chassis_linear_velocity / 10, -rc_ctrl.ch[1] * 0.15);
        PID_calc(&upright_trim_pid, INS_angle[2], 0);
        //trim_pos = - trim_pos_pid.out + upright_trim_pid.out;
        trim_pos = -trim_pos_pid.out;
        trim_pos = (trim_pos - (-100)) * (660 - (-660)) / ((100) - (-100)) - 660;

        //left_trim_speed = PID_calc(&left_trim_speed_pid, left_trim_ecd, rc_ctrl.ch[1]);
        left_trim_speed = PID_calc(&left_trim_speed_pid, left_trim_ecd, trim_pos);

        //right_trim_speed = PID_calc(&right_trim_speed_pid, right_trim_ecd, -rc_ctrl.ch[1]);
        right_trim_speed = PID_calc(&right_trim_speed_pid, right_trim_ecd, -trim_pos);

        left_trim_current = PID_calc(&left_trim_current_pid, motor_measure[2].speed_rpm, left_trim_speed);
        right_trim_current = PID_calc(&right_trim_current_pid, motor_measure[3].speed_rpm, right_trim_speed);

        right_trim_current = limitOutput(right_trim_current, MAX_6020_MOTOR_CAN_CURRENT, MIN_6020_MOTOR_CAN_CURRENT);
        left_trim_current = limitOutput(left_trim_current, MAX_6020_MOTOR_CAN_CURRENT, MIN_6020_MOTOR_CAN_CURRENT);

#if LEFT_WHEEL_REV
        output_left_wheel = -output_left_wheel;
#endif
#if RIGHT_WHEEL_REV
        output_right_wheel = -output_right_wheel;
#endif

        //test messages
        test_speed_pid_out = chassis_pid[0].out;

        // apply current value to motors
        canCmdChassis(left_trim_current * 40, right_trim_current * 40, output_left_wheel * 1.0f, output_right_wheel * 1.0f);
    }
}
