/**
 * @file lv_tutorial_responsive.h
 *
 */

/*
 * -------------------------------------------------
 * See how to create responsive user interfaces
 * ------------------------------------------------
 *
 * Changing the display to different resolution, updating the GUI design
 * or working with dynamic content are much more easier if you use some
 * useful features of the library and follow a few rules.
 *
 * LV_DPI
 * - In lv_conf.h LV_DPI shows how many pixels are there in 1 inch
 * - You should use it as general unit. For example:
 *     lv_obj_set_pos(go_to_red_btn, LV_DPI / 2, LV_DPI);
 * - Built-in styles and themes also use this to set padding and sizes.
 *   So lowering LV_DPI will make paddings smaller.
 * - This way changing to higher pixel density display won't brake your design
 *
 * ALIGN
 * - Use the 'lv_obj_align()' function to align the object relative to each other
 *     lv_obj_align(go_to_red_btn, go_to_blue_btn, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 2, 0);
 * - It helps to keep an arrangement even is an object is moved
 * - the align happens only once when you call the function.
 *
 * AUTO FIT
 * - The container like objects (lv_cont, lv_btn, lv_page) support auto-fit
 * - It means the object's size will automatically set to include all its children
 * - Can be enabled separately horizontally and vertically
 * - It is useful if you have dynamic content
 * - For example a message box will be as high as its text needs
 * - It uses the style.body.padding.hor/ver to make a padding
 * - Auto-fit runs every time when a children changes (not only once when applied)
 *
 * LAYOUT
 * - You can apply a layout on any container like object
 * - It automatically arranges the children according to a policy
 * - For example `lv_list` uses it to put elements below each other
 * - Layout runs every time when a children changes (not only once when applied)
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "lv_tutorial_responsive.hpp"
#include "display/lvgl.h" 
#include "display/lv_hal/lv_hal_disp.h"
#include "display/lv_conf.h" 

// creating a button
lv_obj_t * label1;
lv_obj_t * label2;
lv_obj_t * label3;
lv_obj_t * label4;
lv_obj_t * label5;
lv_obj_t * label6;

lv_obj_t * go_to_red_btn;
lv_obj_t * go_to_blue_btn;
lv_obj_t * go_to_skills_btn;
lv_obj_t * btn3;
lv_obj_t * red_home_btn;
lv_obj_t * btn5;
lv_obj_t * blue_home_btn;
lv_obj_t * scr_color_btn;
lv_obj_t * red_auton_one_btn;
lv_obj_t * red_auton_two_btn;
lv_obj_t * blue_auton_one_btn;
lv_obj_t * blue_auton_two_btn;

lv_obj_t * at_skills_btn;
lv_obj_t * skills_home_btn;



lv_obj_t * homeScreen;
lv_obj_t * redScreen;
lv_obj_t * blueScreen;
lv_obj_t * skillsScreen;

char * red_autons[] = {"Auton 1", "Auton 2"};
char * blue_autons[] = {"Auton 1", "Auton 2"};
char * autons_2d[2][2] = {{"Auton 1", "Auton 2"}, {"Auton 1", "Auton 2"}};

// lv_obj_t* auton_buttons[] = {}
short auton_option;

enum screen_state_type_e screen_color;
char* colors[] = {"UNSELECTED", "RED", "BLUE", "SKILLS"};

static lv_obj_t* button_create( lv_obj_t * parent_screen, lv_coord_t x_pos, lv_coord_t y_pos, lv_coord_t length, lv_coord_t height, const char* label_name) {
    lv_obj_t * btn = lv_btn_create(parent_screen, NULL); //if go_to_red_btn appears on screen2, change to lv_btn_create(screen1)
    lv_obj_set_pos(btn, x_pos, y_pos); 
    lv_obj_set_size(btn, length, height);          /*Use LVDOI to set the size*/

    lv_obj_t * label = lv_label_create(btn, NULL);
    lv_label_set_text(label, label_name);

    return btn;
}

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id useful when there are multiple buttons

    if(id == 0)
    {  
        lv_scr_load(redScreen);
        //dropdown = lv_dropdown_create(redScreen);
        // for (int i = 0; i < sizeof(autons); i++) {
        //     //lv_dropdown_add_option(dropdown, autons[i], i);

        // }
        screen_color = RED;
    }
    else if(id == 1) {
        //lv_label_set_text(label2, "clicked");
        lv_scr_load(blueScreen);
        screen_color = BLUE;
    }
    else if(id == 2){
        lv_scr_load(skillsScreen);
        screen_color = UNSELECTED;
    }

    else if(id == 3) {  
        lv_scr_load(homeScreen);
        screen_color = UNSELECTED;
    }
    else if(id == 4) {
        lv_scr_load(homeScreen);
        screen_color = UNSELECTED;
    }
    else if(id == 5) {  
        lv_scr_load(homeScreen);
        screen_color = UNSELECTED;
    }
    else if(id == 6) {
        auton_option = 1;
    }
    else if (id == 7) {
        auton_option = 2;
    }
    else if (id == 8) {
        auton_option = 1;
    }
    else if (id == 9) {
        auton_option = 2;
    }

    return LV_RES_OK;
}

short get_auton_option() {
    return auton_option;
}

int get_color() {
    return screen_color;
}


// map myMap = {{1, redScreen}, {2, bluescreen}};
// lv_scr_load(myMap[id]);
 
void lv_tutorial_responsive(void)
{
    // adding text
    /*Create a Label on the currently active screen*/
    //label1 =  lv_label_create(lv_scr_act(), NULL);
    /*Modify the Label's text*/
    //lv_label_set_text(label1, "Hello world!");
    /* Align the Label to the center
     * NULL means align on parent (which is the screen now)
     * 0, 0 at the end means an x, y offset after alignment*/
    //lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);
    

    homeScreen = lv_obj_create(NULL, NULL);
    redScreen = lv_obj_create(NULL, NULL);
    blueScreen = lv_obj_create(NULL, NULL);
    skillsScreen = lv_obj_create(NULL, NULL);

    lv_scr_load(homeScreen);


    /*LV_DPI*/
    //scr_color_btn = button_create(lv_scr_act(), LV_DPI, LV_DPI, LV_DPI, LV_DPI/2, colors[screen_color]);    

    // homeScreen
    go_to_red_btn = button_create(homeScreen, LV_DPI - 80, LV_DPI/10, LV_DPI, LV_DPI/2, "RED");
    go_to_blue_btn = button_create(homeScreen, 0, 0, LV_DPI, LV_DPI/2, "BLUE");
    lv_obj_align(go_to_blue_btn, go_to_red_btn, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0);
    go_to_skills_btn = button_create(homeScreen, 0, 0, LV_DPI, LV_DPI/2, "SKILLS");
    lv_obj_align(go_to_skills_btn, go_to_blue_btn, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0);


    // redSceen
    btn3 = button_create(redScreen, LV_DPI - 80, LV_DPI / 10, 200, LV_DPI/2, "MADE IT TO RED");
    red_home_btn = button_create(redScreen, 0, 0, LV_DPI, LV_DPI / 2, "HOME");
    lv_obj_align(red_home_btn, btn3, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0); 

    red_auton_one_btn = button_create(redScreen, LV_DPI - 80, LV_DPI, LV_DPI, LV_DPI/2, red_autons[0]);
    red_auton_two_btn = button_create(redScreen, 0, 0, LV_DPI, LV_DPI/2, red_autons[1]);
    lv_obj_align(red_auton_two_btn, red_auton_one_btn, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0);


    // blueScreen
    btn5 = button_create(blueScreen, LV_DPI - 80, LV_DPI / 10, 200, LV_DPI/2, "MADE IT TO BLUE");
    blue_home_btn = button_create(blueScreen, 0, 0, LV_DPI, LV_DPI / 2, "HOME");
    lv_obj_align(blue_home_btn, btn5, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0);   

    blue_auton_one_btn = button_create(blueScreen, LV_DPI - 80, LV_DPI, LV_DPI, LV_DPI/2, blue_autons[0]);
    blue_auton_two_btn = button_create(blueScreen, 0, 0, LV_DPI, LV_DPI/2, blue_autons[1]);
    lv_obj_align(blue_auton_two_btn, blue_auton_one_btn, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0);

    // skillsScreen
    at_skills_btn = button_create(skillsScreen, LV_DPI - 80, LV_DPI / 10, 200, LV_DPI/2, "MADE IT TO SKILLS");
    skills_home_btn = button_create(skillsScreen, 0, 0, LV_DPI, LV_DPI / 2, "HOME");
    lv_obj_align(skills_home_btn, at_skills_btn, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0);   


    // code for when button is clicked
    lv_obj_set_free_num(go_to_red_btn, 0); // set button is to 0
    lv_obj_set_free_num(go_to_blue_btn, 1); 
    lv_obj_set_free_num(go_to_skills_btn, 2); 
    // lv_obj_set_free_num(btn3, 2); 
    lv_obj_set_free_num(red_home_btn, 3); 
    lv_obj_set_free_num(skills_home_btn, 4); 
    // lv_obj_set_free_num(btn5, 4); 
    lv_obj_set_free_num(blue_home_btn, 5); 

    lv_obj_set_free_num(red_auton_one_btn, 6); 
    lv_obj_set_free_num(red_auton_two_btn, 7); 
    lv_obj_set_free_num(blue_auton_one_btn, 8); 
    lv_obj_set_free_num(blue_auton_one_btn, 9); 

    

    lv_btn_set_action(go_to_red_btn, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_action(go_to_blue_btn, LV_BTN_ACTION_CLICK, btn_click_action);
    // lv_btn_set_action(btn3, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(red_home_btn, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(btn5, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(blue_home_btn, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(go_to_skills_btn, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(skills_home_btn, LV_BTN_ACTION_CLICK, btn_click_action);

    lv_btn_set_action(red_auton_one_btn, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(red_auton_two_btn, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(blue_auton_one_btn, LV_BTN_ACTION_CLICK, btn_click_action);
    lv_btn_set_action(blue_auton_one_btn, LV_BTN_ACTION_CLICK, btn_click_action);

    
}



