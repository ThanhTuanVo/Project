// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"
static void dropdown_event_handler(lv_event_t *e);

void ui_Screen2_screen_init(void)
{
    ui_Screen2 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0xDFDDDD), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_optionprocess = lv_dropdown_create(ui_Screen2);
    lv_dropdown_set_options(ui_optionprocess, "Say khu am\nLen men\nOn dinh\nBao quan");
    lv_obj_add_event_cb(ui_optionprocess, dropdown_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_width(ui_optionprocess, 221);
    lv_obj_set_height(ui_optionprocess, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_optionprocess, -114);
    lv_obj_set_y(ui_optionprocess, -158);
    lv_obj_set_align(ui_optionprocess, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_optionprocess, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_processlabel = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_processlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_processlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_processlabel, -321);
    lv_obj_set_y(ui_processlabel, -158);
    lv_obj_set_align(ui_processlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_processlabel, "Process:");
    lv_obj_set_style_text_font(ui_processlabel, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_templabel1 = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_templabel1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_templabel1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_templabel1, -284);
    lv_obj_set_y(ui_templabel1, -90);
    lv_obj_set_align(ui_templabel1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_templabel1, "Temperature:");
    lv_obj_set_style_text_font(ui_templabel1, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_humdlabel = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_humdlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_humdlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_humdlabel, -309);
    lv_obj_set_y(ui_humdlabel, -19);
    lv_obj_set_align(ui_humdlabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_humdlabel, "Humidity:");
    lv_obj_set_style_text_font(ui_humdlabel, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timelabel = lv_label_create(ui_Screen2);
    lv_obj_set_width(ui_timelabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_timelabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_timelabel, -337);
    lv_obj_set_y(ui_timelabel, 56);
    lv_obj_set_align(ui_timelabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_timelabel, "Time:");
    lv_obj_set_style_text_font(ui_timelabel, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tempsetting1 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_tempsetting1, 175);
    lv_obj_set_height(ui_tempsetting1, 44);
    lv_obj_set_x(ui_tempsetting1, -93);
    lv_obj_set_y(ui_tempsetting1, -88);
    lv_obj_set_align(ui_tempsetting1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_tempsetting1, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_humdsetting1 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_humdsetting1, 175);
    lv_obj_set_height(ui_humdsetting1, 44);
    lv_obj_set_x(ui_humdsetting1, -92);
    lv_obj_set_y(ui_humdsetting1, -19);
    lv_obj_set_align(ui_humdsetting1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_humdsetting1, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_timesetting1 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_timesetting1, 175);
    lv_obj_set_height(ui_timesetting1, 44);
    lv_obj_set_x(ui_timesetting1, -91);
    lv_obj_set_y(ui_timesetting1, 56);
    lv_obj_set_align(ui_timesetting1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_timesetting1, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_tempsetting2 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_tempsetting2, 175);
    lv_obj_set_height(ui_tempsetting2, 44);
    lv_obj_set_x(ui_tempsetting2, -93);
    lv_obj_set_y(ui_tempsetting2, -90);
    lv_obj_set_align(ui_tempsetting2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_tempsetting2, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_humdsetting2 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_humdsetting2, 175);
    lv_obj_set_height(ui_humdsetting2, 44);
    lv_obj_set_x(ui_humdsetting2, -92);
    lv_obj_set_y(ui_humdsetting2, -19);
    lv_obj_set_align(ui_humdsetting2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_humdsetting2, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_timesetting2 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_timesetting2, 175);
    lv_obj_set_height(ui_timesetting2, 44);
    lv_obj_set_x(ui_timesetting2, -91);
    lv_obj_set_y(ui_timesetting2, 56);
    lv_obj_set_align(ui_timesetting2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_timesetting2, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_tempsetting3 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_tempsetting3, 175);
    lv_obj_set_height(ui_tempsetting3, 44);
    lv_obj_set_x(ui_tempsetting3, -93);
    lv_obj_set_y(ui_tempsetting3, -90);
    lv_obj_set_align(ui_tempsetting3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_tempsetting3, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_humdsetting3 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_humdsetting3, 175);
    lv_obj_set_height(ui_humdsetting3, 44);
    lv_obj_set_x(ui_humdsetting3, -92);
    lv_obj_set_y(ui_humdsetting3, -19);
    lv_obj_set_align(ui_humdsetting3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_humdsetting3, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_timesetting3 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_timesetting3, 175);
    lv_obj_set_height(ui_timesetting3, 44);
    lv_obj_set_x(ui_timesetting3, -91);
    lv_obj_set_y(ui_timesetting3, 56);
    lv_obj_set_align(ui_timesetting3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_timesetting3, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_tempsetting4 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_tempsetting4, 175);
    lv_obj_set_height(ui_tempsetting4, 44);
    lv_obj_set_x(ui_tempsetting4, -93);
    lv_obj_set_y(ui_tempsetting4, -90);
    lv_obj_set_align(ui_tempsetting4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_tempsetting4, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_humdsetting4 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_humdsetting4, 175);
    lv_obj_set_height(ui_humdsetting4, 44);
    lv_obj_set_x(ui_humdsetting4, -92);
    lv_obj_set_y(ui_humdsetting4, -19);
    lv_obj_set_align(ui_humdsetting4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_humdsetting4, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_timesetting4 = lv_textarea_create(ui_Screen2);
    lv_obj_set_width(ui_timesetting4, 175);
    lv_obj_set_height(ui_timesetting4, 44);
    lv_obj_set_x(ui_timesetting4, -91);
    lv_obj_set_y(ui_timesetting4, 56);
    lv_obj_set_align(ui_timesetting4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_timesetting4, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_okbnt = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_okbnt, 100);
    lv_obj_set_height(ui_okbnt, 50);
    lv_obj_set_x(ui_okbnt, -295);
    lv_obj_set_y(ui_okbnt, 154);
    lv_obj_set_align(ui_okbnt, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_okbnt, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_okbnt, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_okbnt, lv_color_hex(0x00BA0F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_okbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label13 = lv_label_create(ui_okbnt);
    lv_obj_set_width(ui_Label13, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label13, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label13, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label13, "OK");
    lv_obj_set_style_text_color(ui_Label13, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label13, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label13, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_cancelbnt = lv_btn_create(ui_Screen2);
    lv_obj_set_width(ui_cancelbnt, 100);
    lv_obj_set_height(ui_cancelbnt, 50);
    lv_obj_set_x(ui_cancelbnt, -96);
    lv_obj_set_y(ui_cancelbnt, 154);
    lv_obj_set_align(ui_cancelbnt, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_cancelbnt, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_cancelbnt, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_cancelbnt, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_cancelbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label14 = lv_label_create(ui_cancelbnt);
    lv_obj_set_width(ui_Label14, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label14, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label14, 1);
    lv_obj_set_y(ui_Label14, 1);
    lv_obj_set_align(ui_Label14, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label14, "CANCEL");
    lv_obj_set_style_text_color(ui_Label14, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label14, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label14, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Keyboard2 = lv_keyboard_create(ui_Screen2);
    lv_keyboard_set_mode(ui_Keyboard2, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_width(ui_Keyboard2, 368);
    lv_obj_set_height(ui_Keyboard2, 365);
    lv_obj_set_x(ui_Keyboard2, 213);
    lv_obj_set_y(ui_Keyboard2, -2);
    lv_obj_set_align(ui_Keyboard2, LV_ALIGN_CENTER);

    lv_obj_add_event_cb(ui_tempsetting1, ui_event_tempsetting1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_humdsetting1, ui_event_humdsetting1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_timesetting1, ui_event_timesetting1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_tempsetting2, ui_event_tempsetting2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_humdsetting2, ui_event_humdsetting2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_timesetting2, ui_event_timesetting2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_tempsetting3, ui_event_tempsetting3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_humdsetting3, ui_event_humdsetting3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_timesetting3, ui_event_timesetting3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_tempsetting4, ui_event_tempsetting4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_humdsetting4, ui_event_humdsetting4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_timesetting4, ui_event_timesetting4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_okbnt, ui_event_okbnt, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_cancelbnt, ui_event_cancelbnt, LV_EVENT_ALL, NULL);

    lv_obj_add_event_cb(ui_okbnt, ui_event_okbnt, LV_EVENT_CLICKED, NULL);

}
static void dropdown_event_handler(lv_event_t *e) {
    lv_obj_t *dropdown = lv_event_get_target(e);
    char selected_option[64];
    lv_dropdown_get_selected_str(dropdown, selected_option, sizeof(selected_option));
    //printf("Selected option: %s\n", selected_option);

    if (strcmp(selected_option, "Say khu am") == 0) {
        lv_obj_clear_flag(ui_tempsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_humdsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_timesetting1, LV_OBJ_FLAG_HIDDEN);
        
        lv_obj_add_flag(ui_tempsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting4, LV_OBJ_FLAG_HIDDEN);
        
    } else if (strcmp(selected_option, "Len men") == 0) {
        lv_obj_clear_flag(ui_tempsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_humdsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_timesetting2, LV_OBJ_FLAG_HIDDEN);
     
        lv_obj_add_flag(ui_tempsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting4, LV_OBJ_FLAG_HIDDEN);
    
    } else if (strcmp(selected_option, "On dinh") == 0) {
        lv_obj_clear_flag(ui_tempsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_humdsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_timesetting3, LV_OBJ_FLAG_HIDDEN);

        lv_obj_add_flag(ui_tempsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting4, LV_OBJ_FLAG_HIDDEN);
     
    } else if (strcmp(selected_option, "Bao quan") == 0) {
        lv_obj_clear_flag(ui_tempsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_humdsetting4, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_timesetting4, LV_OBJ_FLAG_HIDDEN);

        lv_obj_add_flag(ui_tempsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting1, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_tempsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_humdsetting3, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_timesetting3, LV_OBJ_FLAG_HIDDEN);
        
    }
}
