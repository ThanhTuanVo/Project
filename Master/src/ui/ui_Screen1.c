// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen1, lv_color_hex(0x89FFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_homelabel = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_homelabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_homelabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_homelabel, -141);
    lv_obj_set_y(ui_homelabel, -213);
    lv_obj_set_align(ui_homelabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_homelabel, "HOME SCREEN");
    lv_obj_set_style_text_font(ui_homelabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui_homelabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_homelabel, lv_color_hex(0x0DF3C8), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_homelabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_homelabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_homelabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_homelabel, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Parameter = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Parameter, 498);
    lv_obj_set_height(ui_Parameter, 325);
    lv_obj_set_x(ui_Parameter, -146);
    lv_obj_set_y(ui_Parameter, -24);
    lv_obj_set_align(ui_Parameter, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Parameter, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_Parameter, lv_color_hex(0x4C4C4C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Parameter, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Parameter, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_giaidoan = lv_label_create(ui_Parameter);
    lv_obj_set_width(ui_giaidoan, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_giaidoan, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_giaidoan, 9);
    lv_obj_set_y(ui_giaidoan, -118);
    lv_obj_set_align(ui_giaidoan, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_giaidoan, "Process:");
    lv_obj_set_style_text_font(ui_giaidoan, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timeelapsed = lv_label_create(ui_Parameter);
    lv_obj_set_width(ui_timeelapsed, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_timeelapsed, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_timeelapsed, -8);
    lv_obj_set_y(ui_timeelapsed, 89);
    lv_obj_set_align(ui_timeelapsed, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_timeelapsed, "Time elapsed:");
    lv_obj_set_style_text_font(ui_timeelapsed, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_timeremaining = lv_label_create(ui_Parameter);
    lv_obj_set_width(ui_timeremaining, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_timeremaining, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_timeremaining, -9);
    lv_obj_set_y(ui_timeremaining, 129);
    lv_obj_set_align(ui_timeremaining, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_timeremaining, "Time remaining:");
    lv_obj_set_style_text_font(ui_timeremaining, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_inputstatus = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_inputstatus, 285);
    lv_obj_set_height(ui_inputstatus, 168);
    lv_obj_set_x(ui_inputstatus, 252);
    lv_obj_set_y(ui_inputstatus, -149);
    lv_obj_set_align(ui_inputstatus, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_inputstatus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_inputstatus, lv_color_hex(0x535353), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_inputstatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_inputstatus, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_wifilabel = lv_label_create(ui_inputstatus);
    lv_obj_set_width(ui_wifilabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_wifilabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_wifilabel, -10);
    lv_obj_set_y(ui_wifilabel, -61);
    lv_obj_set_align(ui_wifilabel, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_wifilabel, "WIFI:");
    lv_obj_set_style_text_font(ui_wifilabel, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_mqttlabel = lv_label_create(ui_inputstatus);
    lv_obj_set_width(ui_mqttlabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_mqttlabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_mqttlabel, 105);
    lv_obj_set_y(ui_mqttlabel, -61);
    lv_obj_set_align(ui_mqttlabel, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_mqttlabel, "MQTT:");
    lv_obj_set_style_text_font(ui_mqttlabel, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_statustemp = lv_label_create(ui_inputstatus);
    lv_obj_set_width(ui_statustemp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_statustemp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_statustemp, -10);
    lv_obj_set_y(ui_statustemp, -32);
    lv_obj_set_align(ui_statustemp, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_statustemp, "Temperature sensor:");
    lv_obj_set_style_text_font(ui_statustemp, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_statushumi = lv_label_create(ui_inputstatus);
    lv_obj_set_width(ui_statushumi, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_statushumi, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_statushumi, -10);
    lv_obj_set_y(ui_statushumi, -9);
    lv_obj_set_align(ui_statushumi, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_statushumi, "Humidity sensor:");
    lv_obj_set_style_text_font(ui_statushumi, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tempalarm = lv_label_create(ui_inputstatus);
    lv_obj_set_width(ui_tempalarm, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tempalarm, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tempalarm, -10);
    lv_obj_set_y(ui_tempalarm, 64);
    lv_obj_set_align(ui_tempalarm, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_tempalarm, "Temp alarm:");
    lv_obj_set_style_text_font(ui_tempalarm, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_poweralarm = lv_label_create(ui_inputstatus);
    lv_obj_set_width(ui_poweralarm, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_poweralarm, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_poweralarm, -10);
    lv_obj_set_y(ui_poweralarm, 39);
    lv_obj_set_align(ui_poweralarm, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_poweralarm, "Power alarm:");
    lv_obj_set_style_text_font(ui_poweralarm, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lossphasealarm = lv_label_create(ui_inputstatus);
    lv_obj_set_width(ui_lossphasealarm, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lossphasealarm, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lossphasealarm, -10);
    lv_obj_set_y(ui_lossphasealarm, 15);
    lv_obj_set_align(ui_lossphasealarm, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_lossphasealarm, "Loss phase alarm:");
    lv_obj_set_style_text_font(ui_lossphasealarm, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_outputstatus = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_outputstatus, 285);
    lv_obj_set_height(ui_outputstatus, 108);
    lv_obj_set_x(ui_outputstatus, 252);
    lv_obj_set_y(ui_outputstatus, -2);
    lv_obj_set_align(ui_outputstatus, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_outputstatus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_border_color(ui_outputstatus, lv_color_hex(0x535353), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_outputstatus, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_outputstatus, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_dehumidifierfan = lv_label_create(ui_outputstatus);
    lv_obj_set_width(ui_dehumidifierfan, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_dehumidifierfan, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_dehumidifierfan, -8);
    lv_obj_set_y(ui_dehumidifierfan, 34);
    lv_obj_set_align(ui_dehumidifierfan, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_dehumidifierfan, "Dehumidifier fan:");
    lv_obj_set_style_text_font(ui_dehumidifierfan, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_humipump = lv_label_create(ui_outputstatus);
    lv_obj_set_width(ui_humipump, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_humipump, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_humipump, -8);
    lv_obj_set_y(ui_humipump, 9);
    lv_obj_set_align(ui_humipump, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_humipump, "Humidity pump:");
    lv_obj_set_style_text_font(ui_humipump, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_convectionfan = lv_label_create(ui_outputstatus);
    lv_obj_set_width(ui_convectionfan, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_convectionfan, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_convectionfan, -8);
    lv_obj_set_y(ui_convectionfan, -15);
    lv_obj_set_align(ui_convectionfan, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_convectionfan, "Convection Fan:");
    lv_obj_set_style_text_font(ui_convectionfan, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_heating = lv_label_create(ui_outputstatus);
    lv_obj_set_width(ui_heating, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_heating, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_heating, -8);
    lv_obj_set_y(ui_heating, -38);
    lv_obj_set_align(ui_heating, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_heating, "Heating:");
    lv_obj_set_style_text_font(ui_heating, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Startbnt = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Startbnt, 100);
    lv_obj_set_height(ui_Startbnt, 50);
    lv_obj_set_x(ui_Startbnt, -326);
    lv_obj_set_y(ui_Startbnt, 186);
    lv_obj_set_align(ui_Startbnt, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Startbnt, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Startbnt, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Startbnt, lv_color_hex(0x00BC12), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Startbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Startbnt, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Startbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Startbnt, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_startLabel = lv_label_create(ui_Startbnt);
    lv_obj_set_width(ui_startLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_startLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_startLabel, -1);
    lv_obj_set_y(ui_startLabel, 0);
    lv_obj_set_align(ui_startLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_startLabel, "START");
    lv_obj_set_style_text_color(ui_startLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_startLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Stopbnt = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Stopbnt, 100);
    lv_obj_set_height(ui_Stopbnt, 50);
    lv_obj_set_x(ui_Stopbnt, -155);
    lv_obj_set_y(ui_Stopbnt, 187);
    lv_obj_set_align(ui_Stopbnt, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Stopbnt, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Stopbnt, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Stopbnt, lv_color_hex(0xFF3232), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Stopbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Stopbnt, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Stopbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Stopbnt, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_stopLabel = lv_label_create(ui_Stopbnt);
    lv_obj_set_width(ui_stopLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_stopLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_stopLabel, -2);
    lv_obj_set_y(ui_stopLabel, 1);
    lv_obj_set_align(ui_stopLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_stopLabel, "STOP");
    lv_obj_set_style_text_color(ui_stopLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_stopLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Settingbnt = lv_btn_create(ui_Screen1);
    lv_obj_set_width(ui_Settingbnt, 100);
    lv_obj_set_height(ui_Settingbnt, 50);
    lv_obj_set_x(ui_Settingbnt, 21);
    lv_obj_set_y(ui_Settingbnt, 186);
    lv_obj_set_align(ui_Settingbnt, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Settingbnt, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Settingbnt, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Settingbnt, lv_color_hex(0xFFFF0C), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Settingbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Settingbnt, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Settingbnt, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Settingbnt, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_settingLabel = lv_label_create(ui_Settingbnt);
    lv_obj_set_width(ui_settingLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_settingLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_settingLabel, -1);
    lv_obj_set_y(ui_settingLabel, 1);
    lv_obj_set_align(ui_settingLabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_settingLabel, "SETTING");
    lv_obj_set_style_text_color(ui_settingLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_settingLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel5 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel5, 226);
    lv_obj_set_height(ui_Panel5, 128);
    lv_obj_set_x(ui_Panel5, -266);
    lv_obj_set_y(ui_Panel5, -40);
    lv_obj_set_align(ui_Panel5, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0xFEBDC3), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_templabel = lv_label_create(ui_Panel5);
    lv_obj_set_width(ui_templabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_templabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_templabel, 1);
    lv_obj_set_y(ui_templabel, -45);
    lv_obj_set_align(ui_templabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_templabel, "Temperature");
    lv_obj_set_style_text_font(ui_templabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_fvtemp = lv_label_create(ui_Panel5);
    lv_obj_set_width(ui_fvtemp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_fvtemp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_fvtemp, -1);
    lv_obj_set_y(ui_fvtemp, -11);
    lv_obj_set_align(ui_fvtemp, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_fvtemp, "FV");
    lv_obj_set_style_text_font(ui_fvtemp, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_pvtemp = lv_label_create(ui_Panel5);
    lv_obj_set_width(ui_pvtemp, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_pvtemp, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_pvtemp, -1);
    lv_obj_set_y(ui_pvtemp, 33);
    lv_obj_set_align(ui_pvtemp, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_pvtemp, "PV");
    lv_obj_set_style_text_font(ui_pvtemp, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel6 = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_Panel6, 226);
    lv_obj_set_height(ui_Panel6, 128);
    lv_obj_set_x(ui_Panel6, -25);
    lv_obj_set_y(ui_Panel6, -40);
    lv_obj_set_align(ui_Panel6, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel6, lv_color_hex(0xA2C2F4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_humilabel = lv_label_create(ui_Panel6);
    lv_obj_set_width(ui_humilabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_humilabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_humilabel, -1);
    lv_obj_set_y(ui_humilabel, -45);
    lv_obj_set_align(ui_humilabel, LV_ALIGN_CENTER);
    lv_label_set_text(ui_humilabel, "Humidity");
    lv_obj_set_style_text_font(ui_humilabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_fvhumi = lv_label_create(ui_Panel6);
    lv_obj_set_width(ui_fvhumi, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_fvhumi, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_fvhumi, -5);
    lv_obj_set_y(ui_fvhumi, -10);
    lv_obj_set_align(ui_fvhumi, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_fvhumi, "FV");
    lv_obj_set_style_text_font(ui_fvhumi, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_pvhumi = lv_label_create(ui_Panel6);
    lv_obj_set_width(ui_pvhumi, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_pvhumi, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_pvhumi, -5);
    lv_obj_set_y(ui_pvhumi, 33);
    lv_obj_set_align(ui_pvhumi, LV_ALIGN_LEFT_MID);
    lv_label_set_text(ui_pvhumi, "PV");
    lv_obj_set_style_text_font(ui_pvhumi, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Chart2 = lv_chart_create(ui_Screen1);
    lv_obj_set_width(ui_Chart2, 222);
    lv_obj_set_height(ui_Chart2, 154);
    lv_obj_set_x(ui_Chart2, 252);
    lv_obj_set_y(ui_Chart2, 134);
    lv_obj_set_align(ui_Chart2, LV_ALIGN_CENTER);
    lv_chart_set_type(ui_Chart2, LV_CHART_TYPE_LINE);
    // lv_chart_set_axis_tick(ui_Chart2, LV_CHART_AXIS_PRIMARY_X, 10, 5, 5, 2, true, 50);
    // lv_chart_set_axis_tick(ui_Chart2, LV_CHART_AXIS_PRIMARY_Y, 10, 5, 5, 2, true, 50);
    // lv_chart_set_axis_tick(ui_Chart2, LV_CHART_AXIS_SECONDARY_Y, 10, 5, 5, 2, true, 25);
    lv_chart_series_t * ui_Chart2_series_1 = lv_chart_add_series(ui_Chart2, lv_color_hex(0x808080),
                                                                 LV_CHART_AXIS_PRIMARY_Y);
    static lv_coord_t ui_Chart2_series_1_array[] = { 0, 10, 20, 40, 80, 80, 40, 20, 10, 0 };
    lv_chart_set_ext_y_array(ui_Chart2, ui_Chart2_series_1, ui_Chart2_series_1_array);

    lv_obj_add_event_cb(ui_Startbnt, ui_event_Startbnt, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Settingbnt, ui_event_Settingbnt, LV_EVENT_ALL, NULL);

}