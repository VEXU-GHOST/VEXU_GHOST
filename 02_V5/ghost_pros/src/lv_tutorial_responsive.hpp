/**
 * @file lv_tutorial_responsive.h
 *
 */

#ifndef LV_TUTORIAL_RESPONSIVE_H
#define LV_TUTORIAL_RESPONSIVE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef LV_CONF_INCLUDE_SIMPLE
#include "lvgl.h"
#include "lv_ex_conf.h"
#else
//#include "../../../lvgl/lvgl.h"
//#include "../../../lv_ex_conf.h"
#endif



/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/


//short color_chosen; // 0 = unselected, 1 = red, 2 = blue, 3 = skills
// short auton_option; // 0 = none, otherwise is the index from the array of autons + 1
// make get method to access auton_option

enum screen_state_type_e {
    UNSELECTED = 0b00,
    RED = 0b01,
    BLUE = 0b10,
    SKILLS = 0b11
};




void lv_tutorial_responsive(void);
short get_auton_option();
int get_color();





/**********************
 *      MACROS
 **********************/

#endif /*USE_LV_TUTORIALS*/

#ifdef __cplusplus
} /* extern "C" */

#endif /*LV_TUTORIAL_ANTMATION_H*/