/**
 * @file lv_tutorial_responsive.hpp
 */

#ifndef LV_TUTORIAL_RESPONSIVE_HPP
#define LV_TUTORIAL_RESPONSIVE_HPP

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
// #include "../../../lvgl/lvgl.h"
// #include "../../../lv_ex_conf.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

// Represents the screen state (color selection)
enum screen_state_type_e {
    UNSELECTED = 0b00,
    RED        = 0b01,
    BLUE       = 0b10,
    SKILLS     = 0b11
};

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_tutorial_responsive(void);
short get_auton_option(void);
int get_color(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LV_TUTORIAL_RESPONSIVE_HPP */
