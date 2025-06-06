// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"
char wifi_ssid[32];
char wifi_password[32];
///////////////////// VARIABLES ////////////////////

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_homelabel;
lv_obj_t * ui_Parameter;
lv_obj_t * ui_giaidoan;
lv_obj_t * ui_timeelapsed;
lv_obj_t * ui_timeremaining;
lv_obj_t * ui_inputstatus;
lv_obj_t * ui_wifilabel;
lv_obj_t * ui_mqttlabel;
lv_obj_t * ui_statustemp;
lv_obj_t * ui_statushumi;
lv_obj_t * ui_tempalarm;
lv_obj_t * ui_poweralarm;
lv_obj_t * ui_lossphasealarm;
lv_obj_t * ui_outputstatus;
lv_obj_t * ui_dehumidifierfan;
lv_obj_t * ui_humipump;
lv_obj_t * ui_convectionfan;
lv_obj_t * ui_heating;
void ui_event_Startbnt(lv_event_t * e);
lv_obj_t * ui_Startbnt;
lv_obj_t * ui_startLabel;
lv_obj_t * ui_Stopbnt;
lv_obj_t * ui_stopLabel;
void ui_event_Settingbnt(lv_event_t * e);
lv_obj_t * ui_Settingbnt;
lv_obj_t * ui_settingLabel;
lv_obj_t * ui_Panel5;
lv_obj_t * ui_templabel;
lv_obj_t * ui_fvtemp;
lv_obj_t * ui_pvtemp;
lv_obj_t * ui_Panel6;
lv_obj_t * ui_humilabel;
lv_obj_t * ui_fvhumi;
lv_obj_t * ui_pvhumi;
lv_obj_t * ui_Chart2;
// CUSTOM VARIABLES

// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
lv_obj_t * ui_Screen2;
lv_obj_t * ui_optionprocess;
lv_obj_t * ui_processlabel;
lv_obj_t * ui_templabel1;
lv_obj_t * ui_humdlabel;
lv_obj_t * ui_timelabel;
void ui_event_tempsetting1(lv_event_t * e);
lv_obj_t * ui_tempsetting1;
void ui_event_humdsetting1(lv_event_t * e);
lv_obj_t * ui_humdsetting1;
void ui_event_timesetting1(lv_event_t * e);
lv_obj_t * ui_timesetting1;
void ui_event_tempsetting2(lv_event_t * e);
lv_obj_t * ui_tempsetting2;
void ui_event_humdsetting2(lv_event_t * e);
lv_obj_t * ui_humdsetting2;
void ui_event_timesetting2(lv_event_t * e);
lv_obj_t * ui_timesetting2;
void ui_event_tempsetting3(lv_event_t * e);
lv_obj_t * ui_tempsetting3;
void ui_event_humdsetting3(lv_event_t * e);
lv_obj_t * ui_humdsetting3;
void ui_event_timesetting3(lv_event_t * e);
lv_obj_t * ui_timesetting3;
void ui_event_tempsetting4(lv_event_t * e);
lv_obj_t * ui_tempsetting4;
void ui_event_humdsetting4(lv_event_t * e);
lv_obj_t * ui_humdsetting4;
void ui_event_timesetting4(lv_event_t * e);
lv_obj_t * ui_timesetting4;
void ui_event_okbnt(lv_event_t * e);
lv_obj_t * ui_okbnt;
lv_obj_t * ui_Label13;
void ui_event_cancelbnt(lv_event_t * e);
lv_obj_t * ui_cancelbnt;
lv_obj_t * ui_Label14;
lv_obj_t * ui_Keyboard2;
// CUSTOM VARIABLES

// SCREEN: ui_Screen3
void ui_Screen3_screen_init(void);
lv_obj_t * ui_Screen3;
void ui_event_parameterbnt(lv_event_t * e);
lv_obj_t * ui_parameterbnt;
lv_obj_t * ui_Label15;
void ui_event_wifibnt(lv_event_t * e);
lv_obj_t * ui_wifibnt;
lv_obj_t * ui_Label2;
lv_obj_t * ui_historybnt;
lv_obj_t * ui_Label3;
void ui_event_backbnt(lv_event_t * e);
lv_obj_t * ui_backbnt;
// CUSTOM VARIABLES

// SCREEN: ui_Screen4
void ui_Screen4_screen_init(void);
lv_obj_t * ui_Screen4;
void ui_event_Panel1(lv_event_t * e);
lv_obj_t * ui_Panel1;
lv_obj_t * ui_Label1;
lv_obj_t * ui_Label5;
void ui_event_idtext(lv_event_t * e);
lv_obj_t * ui_idtext;
void ui_event_passtext(lv_event_t * e);
lv_obj_t * ui_passtext;
lv_obj_t * ui_statuswf;
lv_obj_t * ui_wifiname;
lv_obj_t * ui_Panel2;
lv_obj_t * ui_hmiupdatebnt;
lv_obj_t * ui_Label6;
lv_obj_t * ui_hmiversion;
lv_obj_t * ui_statusupdate1;
lv_obj_t * ui_Panel4;
lv_obj_t * ui_programupdatebnt;
lv_obj_t * ui_Label7;
lv_obj_t * ui_statusupdate2;
lv_obj_t * ui_programversion;
void ui_event_Button5(lv_event_t * e);
lv_obj_t * ui_Button5;
lv_obj_t * ui_Label8;
lv_obj_t * ui_Keyboard1;
// CUSTOM VARIABLES

// EVENTS
lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
// void ui_event_Startbnt(lv_event_t * e)
// {
//     lv_event_code_t event_code = lv_event_get_code(e);

//     if(event_code == LV_EVENT_RELEASED) {
//         (e);
//     }
// }

void ui_event_Settingbnt(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_screen_delete(&ui_Screen1);
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, &ui_Screen3_screen_init);
    }
}

void ui_event_tempsetting1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_tempsetting1);
    }
}

void ui_event_humdsetting1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_humdsetting1);
    }
}

void ui_event_timesetting1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_timesetting1);
    }
}

void ui_event_tempsetting2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_tempsetting2);
    }
}

void ui_event_humdsetting2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_humdsetting2);
    }
}

void ui_event_timesetting2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_timesetting2);
    }
}

void ui_event_tempsetting3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_tempsetting3);
    }
}

void ui_event_humdsetting3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_humdsetting3);
    }
}

void ui_event_timesetting3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_timesetting3);
    }
}

void ui_event_tempsetting4(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_tempsetting4);
    }
}

void ui_event_humdsetting4(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_humdsetting4);
    }
}

void ui_event_timesetting4(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard2,  ui_timesetting4);
    }
}

// void ui_event_okbnt(lv_event_t * e)
// {
//     lv_event_code_t event_code = lv_event_get_code(e);

//     if(event_code == LV_EVENT_RELEASED) {
//         _ui_screen_delete(&ui_Screen2);
//         _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, &ui_Screen3_screen_init);
//     }
// }

void ui_event_cancelbnt(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_delete(&ui_Screen2);
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 0, &ui_Screen3_screen_init);
    }
}

void ui_event_parameterbnt(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_screen_delete(&ui_Screen3);
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 0, &ui_Screen2_screen_init);
    }
}

void ui_event_wifibnt(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_screen_delete(&ui_Screen4);
        _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_Screen4_screen_init);
    }
}

void ui_event_backbnt(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_screen_delete(&ui_Screen3);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 0, &ui_Screen1_screen_init);
    }
}

void ui_event_Panel1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void ui_event_idtext(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_keyboard_set_target(ui_Keyboard1,  ui_idtext);
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
}

void ui_event_passtext(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_keyboard_set_target(ui_Keyboard1,  ui_passtext);
    }
}

void ui_event_Button5(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_RELEASED) {
        _ui_screen_delete(&ui_Screen4);
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_Screen3_screen_init);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Screen2_screen_init();
    ui_Screen3_screen_init();
    ui_Screen4_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen1);
}
void ui_event_KeyBoard1(lv_event_t * e){
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * keyboard = lv_event_get_target(e);
    if (event_code == LV_EVENT_READY) //tick symbol
    {
        snprintf(wifi_ssid, sizeof(wifi_ssid), "%s", lv_textarea_get_text(ui_idtext));
        snprintf(wifi_password, sizeof(wifi_password), "%s", lv_textarea_get_text(ui_passtext));
        //printf("DEBUG: SSID=%s, PASS=%s\n", wifi_ssid, wifi_password);
    }
}

 const char * get_wifi_ssid() {
    return wifi_ssid;
}

const char * get_wifi_password() {
    return wifi_password;
}