#include <stdio.h>

/* DEFINITIONS */
/*
#define dW 1800
#define dH 1000
#define click_force_coeff 50
#define FONTHEIGHT 8
#define DEBUG false

#define XL  0.5         //  position boundaries
#define RXP  +3         // reward value for being good
#define RXL  -5         // reward value for going out of boundaries
#define VL  0.5         //  speed boundaries
#define RVP +3          // reward value for being good
#define RVL -5          // reward value for going too fast
#define T1  0.01745     //  PI/180
#define T6  0.10472     //  6*PI/180
#define RT0 +3          // reward value for being good
#define RT1 -5          // reward value for being sligthly inclined
#define RT6 -8          // reward value for being highly inclined
#define W50   0.87266   //  50*PI/180
#define RW0   +3        // reward value for being good
#define RW50  -5        // reward value for too high angoular velocity
#define FAIL_REWARD -50 // reward value for failing
*/

/* VARIABLES DECLARATIONS */

typedef struct {
  float x_c1;       // x position ball camera 1
  float y_c1;       // y position ball camera 1
  float x_c2;		// x position ball camera 2
  float y_c2;		// y position ball camera 2 
} STATE;

typedef struct {
  int w_display;   // display width
  int h_display;   // display height
  int edge;        // display edge
} DISPLAY;

typedef struct Button {
    float x, y, width, height;
    const char* text;
    ALLEGRO_COLOR color;
} BUTTON;

typedef struct {
    BUTTON all_buttons[4];
} ALL_BUTTONS;

typedef struct {
    float x, y, width, height;
    float value; // valore dello slider tra 0.0 e 1.0
    bool dragging;
} Slider;

typedef struct {
    Slider all_sliders[3];
} all_sliders;

STATE ball_state = {1, 0, 965, 0};
DISPLAY disp = {1920, 1080, 5};
BUTTON load = { 100, 800, 250, 50, "LOAD", al_map_rgb(0, 0, 255) };
BUTTON save = { 400, 800, 250, 50, "SAVE", al_map_rgb(0, 0, 255) };
BUTTON button_start_simulation = { 700, 800, 250, 50, "START SIMULATION", al_map_rgb(0, 0, 255) };
BUTTON button_start_training = { 1000, 800, 250, 50, "START TRAINING", al_map_rgb(0, 0, 255) };
Slider epsilon_slider = { 50, disp.h_display - 200, disp.w_display - 100, 20, 0.5, false };
Slider alpha_slider = { 50, disp.h_display - 100, disp.w_display - 100, 20, 0.5, false };
Slider gamma_slider = { 50, disp.h_display - 50, disp.w_display - 100, 20, 0.5, false };

// variables training algorithm 
#define REWARD_NEGATIVE -20
#define REWARD_POSITIVE +100
#define BXW 5 // number box width
#define BXH 3 // number box height
float Q[BXW * BXH * 4];
float R[BXW * BXH * 4];
int central_box = (BXW * BXH -1) / 2; // NOTE: it works only if the number of boxes is odd!!

float box_width = (disp.w_display / 2 - disp.edge) / BXW;
float box_height = (disp.h_display / 2) / BXH;
float pos_center[2] = { (float(disp.w_display) / 2 - float(disp.edge)) / 2, (float(disp.h_display) / 2) / 2 };

