#ifndef AH_CONTROL_TABLE_H
#define AH_CONTROL_TABLE_H

#include <stdint.h>

enum control_table_addr{
    OPERATING_MODE_ADDR = 0,
    GOAL_POS_ADDR = 1,
    GOAL_VEL_ADDR = 2,
    GOAL_PWM_ADDR = 3,
    CURRENT_POS_ADDR = 4,
    CURRENT_SPEED_ADDR = 5,
    POS_P_ADDR = 6,
    POS_I_ADDR = 7,
    POS_D_ADDR = 8,
    VEL_P_ADDR = 9,
    VEL_I_ADDR = 10,
    VEL_D_ADDR = 11,
};

#endif