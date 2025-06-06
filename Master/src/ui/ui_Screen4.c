// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen4_screen_init(void)
{
    ui_Screen4 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen4, lv_color_hex(0xD5D5D5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel1 = lv_obj_create(ui_Screen4);
    lv_obj_set_width(ui_Panel1, 776);
    lv_obj_set_height(ui_Panel1, 295);
    lv_obj_set_x(ui_Panel1, -1);
    lv_obj_set_y(ui_Panel1, -82);
    lv_obj_set_align(ui_Panel1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_Panel1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label1 = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -235);
    lv_obj_set_y(ui_Label1, -28);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "SSID");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0x00FF22), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_28, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label5 = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_Label5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label5, -233);
    lv_obj_set_y(ui_Label5, 41);
    lv_obj_set_align(ui_Label5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label5, "PASSWORD");
    lv_obj_set_style_text_color(ui_Label5, lv_color_hex(0xFF2525), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label5, &lv_font_montserrat_28, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_idtext = lv_textarea_create(ui_Panel1);
    lv_obj_set_width(ui_idtext, 255);
    lv_obj_set_height(ui_idtext, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_x(ui_idtext, 4);
    lv_obj_set_y(ui_idtext, -30);
    lv_obj_set_align(ui_idtext, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_idtext, "WIFI NAME");
    lv_textarea_set_one_line(ui_idtext, true);
    lv_obj_set_style_text_font(ui_idtext, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_passtext = lv_textarea_create(ui_Panel1);
    lv_obj_set_width(ui_passtext, 255);
    lv_obj_set_height(ui_passtext, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_x(ui_passtext, 4);
    lv_obj_set_y(ui_passtext, 41);
    lv_obj_set_align(ui_passtext, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_passtext, "PASSWORD");
    lv_textarea_set_one_line(ui_passtext, true);
    lv_textarea_set_password_mode(ui_passtext, true);
    lv_obj_set_style_text_font(ui_passtext, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_statuswf = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_statuswf, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_statuswf, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_statuswf, -3);
    lv_obj_set_y(ui_statuswf, -109);
    lv_obj_set_align(ui_statuswf, LV_ALIGN_CENTER);
    lv_label_set_text(ui_statuswf, "Not connect");
    lv_obj_set_style_text_font(ui_statuswf, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_wifiname = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_wifiname, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_wifiname, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_wifiname, 281);
    lv_obj_set_y(ui_wifiname, -111);
    lv_obj_set_align(ui_wifiname, LV_ALIGN_CENTER);
    lv_label_set_text(ui_wifiname, "wifi name");
    lv_obj_set_style_text_font(ui_wifiname, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel2 = lv_obj_create(ui_Screen4);
    lv_obj_set_width(ui_Panel2, 381);
    lv_obj_set_height(ui_Panel2, 161);
    lv_obj_set_x(ui_Panel2, -198);
    lv_obj_set_y(ui_Panel2, 152);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_Panel2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_hmiupdatebnt = lv_btn_create(ui_Panel2);
    lv_obj_set_width(ui_hmiupdatebnt, 133);
    lv_obj_set_height(ui_hmiupdatebnt, 65);
    lv_obj_set_x(ui_hmiupdatebnt, -78);
    lv_obj_set_y(ui_hmiupdatebnt, -1);
    lv_obj_set_align(ui_hmiupdatebnt, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_hmiupdatebnt, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_hmiupdatebnt, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_hmiupdatebnt, lv_color_hex(0xFDF474), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_hmiupdatebnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_hmiupdatebnt, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_hmiupdatebnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_hmiupdatebnt, 3, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label6 = lv_label_create(ui_hmiupdatebnt);
    lv_obj_set_width(ui_Label6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label6, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label6, "HMI Update");
    lv_obj_set_style_text_color(ui_Label6, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label6, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_hmiversion = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_hmiversion, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_hmiversion, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_hmiversion, 56);
    lv_obj_set_y(ui_hmiversion, 17);
    lv_obj_set_align(ui_hmiversion, LV_ALIGN_CENTER);
    lv_label_set_text(ui_hmiversion, "Version:");
    lv_obj_set_style_text_color(ui_hmiversion, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_hmiversion, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_hmiversion, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_statusupdate1 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_statusupdate1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_statusupdate1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_statusupdate1, 47);
    lv_obj_set_y(ui_statusupdate1, -15);
    lv_obj_set_align(ui_statusupdate1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_statusupdate1, "Failed");
    lv_obj_set_style_text_color(ui_statusupdate1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_statusupdate1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_statusupdate1, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel4 = lv_obj_create(ui_Screen4);
    lv_obj_set_width(ui_Panel4, 377);
    lv_obj_set_height(ui_Panel4, 161);
    lv_obj_set_x(ui_Panel4, 199);
    lv_obj_set_y(ui_Panel4, 152);
    lv_obj_set_align(ui_Panel4, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_Panel4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Panel4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_programupdatebnt = lv_btn_create(ui_Panel4);
    lv_obj_set_width(ui_programupdatebnt, 133);
    lv_obj_set_height(ui_programupdatebnt, 65);
    lv_obj_set_x(ui_programupdatebnt, -78);
    lv_obj_set_y(ui_programupdatebnt, -1);
    lv_obj_set_align(ui_programupdatebnt, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_programupdatebnt, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_programupdatebnt, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_programupdatebnt, lv_color_hex(0xFF92C2), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_programupdatebnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_programupdatebnt, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_programupdatebnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_programupdatebnt, 3, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label7 = lv_label_create(ui_programupdatebnt);
    lv_obj_set_width(ui_Label7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label7, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label7, "Program\n Update");
    lv_obj_set_style_text_color(ui_Label7, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label7, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_statusupdate2 = lv_label_create(ui_Panel4);
    lv_obj_set_width(ui_statusupdate2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_statusupdate2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_statusupdate2, 54);
    lv_obj_set_y(ui_statusupdate2, -14);
    lv_obj_set_align(ui_statusupdate2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_statusupdate2, "Failed");
    lv_obj_set_style_text_color(ui_statusupdate2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_statusupdate2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_statusupdate2, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_programversion = lv_label_create(ui_Panel4);
    lv_obj_set_width(ui_programversion, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_programversion, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_programversion, 63);
    lv_obj_set_y(ui_programversion, 17);
    lv_obj_set_align(ui_programversion, LV_ALIGN_CENTER);
    lv_label_set_text(ui_programversion, "Version:");
    lv_obj_set_style_text_color(ui_programversion, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_programversion, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_programversion, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button5 = lv_btn_create(ui_Screen4);
    lv_obj_set_width(ui_Button5, 100);
    lv_obj_set_height(ui_Button5, 50);
    lv_obj_set_x(ui_Button5, -328);
    lv_obj_set_y(ui_Button5, -194);
    lv_obj_set_align(ui_Button5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button5, lv_color_hex(0x282828), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label8 = lv_label_create(ui_Button5);
    lv_obj_set_width(ui_Label8, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label8, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label8, "BACK");
    lv_obj_set_style_text_font(ui_Label8, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Keyboard1 = lv_keyboard_create(ui_Screen4);
    lv_obj_set_width(ui_Keyboard1, 794);
    lv_obj_set_height(ui_Keyboard1, 233);
    lv_obj_set_x(ui_Keyboard1, 0);
    lv_obj_set_y(ui_Keyboard1, 121);
    lv_obj_set_align(ui_Keyboard1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN);     /// Flags

    lv_obj_add_event_cb(ui_idtext, ui_event_idtext, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_passtext, ui_event_passtext, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Panel1, ui_event_Panel1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button5, ui_event_Button5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Keyboard1, ui_event_KeyBoard1, LV_EVENT_ALL, NULL);

}
