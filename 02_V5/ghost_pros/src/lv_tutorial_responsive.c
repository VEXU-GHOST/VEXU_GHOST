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
 *     lv_obj_set_pos(btn1, LV_DPI / 2, LV_DPI);
 * - Built-in styles and themes also use this to set padding and sizes.
 *   So lowering LV_DPI will make paddings smaller.
 * - This way changing to higher pixel density display won't brake your design
 *
 * ALIGN
 * - Use the 'lv_obj_align()' function to align the object relative to each other
 *     lv_obj_align(btn1, btn2, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 2, 0);
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

#include "lv_tutorial_responsive.h"
#include "display/lvgl.h"

     
// creating a button
lv_obj_t * label1;
lv_obj_t * label2;
lv_obj_t * label3;
lv_obj_t * label4;

lv_obj_t * btn1;
lv_obj_t * btn2;

lv_obj_t * btn3;

lv_obj_t * screen1;
lv_obj_t * screen2;

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); //id useful when there are multiple buttons

    if(id == 0)
    {
        // lv_disp_load_screen(screen2);
        // lv_label_set_text(label1, "clicked");      
        lv_scr_load(screen2);  
    }
    if(id == 1) lv_label_set_text(label2, "clicked");

    return LV_RES_OK;
}
 
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
    
    



    /*LV_DPI*/
    lv_obj_create(screen1, NULL);
    lv_obj_create(screen2, NULL);
    //screen2 = lv_obj_create(NULL);

    // lv_disp_load_scr(screen1); //makes screen1 the active screen
    // lv_disp_set_default(screen1);
    // lv_disp_load_scr(screen1);
    // lv_disp_set_default(disp);
    // lv_disp_get_scr_act(disp);


    btn3 = lv_btn_create(lv_scr_act(), screen2); // CHANGE TO ONLY SHOW ON SCREEN2 lv_btn_create(screen2, NULL)
    lv_obj_set_pos(btn3, LV_DPI - 80, LV_DPI / 10); 
    lv_obj_set_size(btn3, LV_DPI, LV_DPI / 2);     
    label3 = lv_label_create(btn3, NULL);
    lv_label_set_text(label3, "MADE IT TO RED");


    btn1 = lv_btn_create(lv_scr_act(), NULL); //if btn1 appears on screen2, change to lv_btn_create(screen1)
    //lv_obj_set_pos(btn1, LV_DPI / 10, LV_DPI / 10);     /*Use LV_DPI to set the position*/
    lv_obj_set_pos(btn1, LV_DPI - 80, LV_DPI / 10); 
    lv_obj_set_size(btn1, LV_DPI, LV_DPI / 2);          /*Use LVDOI to set the size*/

    label1 = lv_label_create(btn1, NULL);
    lv_label_set_text(label1, "RED");

    /*ALIGN*/
    
    btn2 = lv_btn_create(lv_scr_act(), btn1);
    lv_obj_align(btn2, btn1, LV_ALIGN_OUT_RIGHT_MID, LV_DPI / 4, 0);

    label2 = lv_label_create(btn2, NULL);
    lv_label_set_text(label2, "BLUE");


    // code for when button is clicked
    lv_obj_set_free_num(btn1, 0); // set button is to 0
    lv_obj_set_free_num(btn2, 1); 
    lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, btn_click_action);


}

