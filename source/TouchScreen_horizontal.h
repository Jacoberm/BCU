// generated by Fast Light User Interface Designer (fluid) version 1.0302

#ifndef TouchScreen_horizontal_h
#define TouchScreen_horizontal_h
#include "bcu_function.h"
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
//#include "ts_head_file.h"
extern Fl_Double_Window *wd_touch_screen;
#include <FL/Fl_Wizard.H>
extern Fl_Wizard *wz_window_view;
#include <FL/Fl_Group.H>
extern Fl_Group *gp_window_black_screen;
#include <FL/Fl_Tile.H>
extern Fl_Tile *title_ddu_op;
extern Fl_Wizard *wz_select_window;
extern Fl_Group *gp_select_black;
extern Fl_Tile *key_request;
extern Fl_Tile *information;
extern Fl_Tile *start_station;
extern Fl_Tile *end_station;
extern Fl_Tile *current_station;
extern Fl_Tile *next_station;
extern Fl_Tile *open_door;
extern Fl_Tile *end_value;
extern Fl_Tile *start_value;
extern Fl_Tile *open_value;
extern Fl_Tile *next_value;
extern Fl_Tile *current_value;
extern Fl_Wizard *sw_2;
extern Fl_Group *cd_gp_live;
extern Fl_Group *cd_gp_intercomm;
extern Fl_Wizard *cd_wz_window_view;
extern Fl_Tile *pcu_recept;
extern Fl_Tile *pcu_value;

int touch_screen_main();
void TouchScreenMain(int a, int b);
extern int switchFlag;
#endif
