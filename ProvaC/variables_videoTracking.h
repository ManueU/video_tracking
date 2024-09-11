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
typedef struct { // struct assi 
    int x_1;       
    int y_1;  
    int x_2;
    int y_2;
} AXES;

typedef struct {
  float x_1;       // x position ball 1
  float y_1;       // y position ball 1
  float x_2;       // x position ball 2 (trajectory)
  float y_2;       // y position ball 2 (trajectory)
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

//STATE ball_state = {1, 0, 965, 0};
STATE ball_state = {480, 270, 0, 0};
DISPLAY disp = {1920, 1080, 5};
BUTTON load = { 100, 600, 250, 50, "LOAD", al_map_rgb(0, 0, 255) };
BUTTON save = { 100, 650, 250, 50, "SAVE", al_map_rgb(0, 0, 255) };
BUTTON button_start_simulation = { 100, 700, 250, 50, "START SIMULATION", al_map_rgb(0, 0, 255) };
BUTTON button_start_training = { 100, 750, 250, 50, "START TRAINING", al_map_rgb(0, 0, 255) };
Slider epsilon_slider = { 50, disp.h_display - 200, disp.w_display - 100, 20, 0.5, false };
Slider alpha_slider = { 50, disp.h_display - 100, disp.w_display - 100, 20, 0.5, false };
Slider gamma_slider = { 50, disp.h_display - 50, disp.w_display - 100, 20, 0.5, false };

// variables trajectory
int trajectory_ID = 0;
AXES trajectory_ax_x = {disp.w_display / 2 + disp.edge,  disp.h_display / 2, disp.w_display, disp.h_display / 2 };
AXES trajectory_ax_y = {disp.w_display / 2 + disp.edge,  disp.h_display / 2, disp.w_display / 2 + disp.edge,1 };
float amplitude = 100;
float frequency = 0.01;
float phase = 0;

float x_plot[955], y_plot[955];

// variables results 
AXES ax_totreward_x = {disp.w_display*3/8, disp.h_display*3/4 - disp.edge, disp.w_display*5/8,  disp.h_display * 3 / 4 - disp.edge };
AXES ax_totreward_y = { disp.w_display * 3 / 8, disp.h_display * 3 / 4 - disp.edge, disp.w_display * 3 / 8,  disp.h_display/2 + disp.edge };

AXES ax_framecount_x = { disp.w_display * 3 / 8, disp.h_display - disp.edge, disp.w_display * 5 / 8,  disp.h_display - disp.edge };
AXES ax_framecount_y = { disp.w_display * 3 / 8, disp.h_display * 3 / 4 + disp.edge, disp.w_display * 3 / 8,  disp.h_display - disp.edge };

// variables camera
float camera_pan = 0.0, camera_tilt = 0.0;


// variables training algorithm 
float tot_reward = 0; 
int episode_target = 2500; 
int steps_target = 1000; 
#define REWARD_NEGATIVE -20
#define REWARD_NEUTRAL -10
#define REWARD_POSITIVE +1000
#define BXW 5 // number box width
#define BXH 3 // number box height
float Q[BXW * BXH * 4];
float R[BXW * BXH * 4];
int central_box = (BXW * BXH -1) / 2; // NOTE: it works only if the number of boxes is odd!!

float box_width = (disp.w_display / 2 - disp.edge) / BXW;
float box_height = (disp.h_display / 2) / BXH;
float pos_center[2] = { (float(disp.w_display) / 2 - float(disp.edge)) / 2, (float(disp.h_display) / 2) / 2 };

