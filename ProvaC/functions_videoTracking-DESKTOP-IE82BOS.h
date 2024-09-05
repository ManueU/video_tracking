
/* FUNCTIONS DECLARATIONS */


bool is_mouse_over_button(BUTTON btn, int mouse_x, int mouse_y) {
    return mouse_x > btn.x && mouse_x < btn.x + btn.width &&
        mouse_y > btn.y && mouse_y < btn.y + btn.height;
}

int button_clicked(ALL_BUTTONS btn, int mouse_x, int mouse_y) {
    
    int output = 100;
    size_t length = sizeof(btn.all_buttons) / sizeof(btn.all_buttons[0]);
    for(int i = 0; i<length; i++)
    {
        if (mouse_x > btn.all_buttons[i].x && mouse_x < btn.all_buttons[i].x + btn.all_buttons[i].width &&
            mouse_y > btn.all_buttons[i].y && mouse_y < btn.all_buttons[i].y + btn.all_buttons[i].height)
        {
           output = i;
        }
    }
    return output;
}


void draw_button(BUTTON btn, ALLEGRO_FONT* font, ALLEGRO_COLOR color)
{
    al_draw_filled_rectangle(btn.x, btn.y, btn.x + btn.width, btn.y + btn.height, color);
    al_draw_text(font, al_map_rgb(255, 255, 255), btn.x + btn.width / 2, btn.y + 10, ALLEGRO_ALIGN_CENTER, btn.text);

}


void draw_slider(Slider slider) {
    al_draw_filled_rectangle(slider.x, slider.y, slider.x + slider.width, slider.y + slider.height, al_map_rgb(200, 200, 200));
    float handle_x = slider.x + slider.value * slider.width;
    al_draw_filled_rectangle(handle_x - 5, slider.y - 5, handle_x + 5, slider.y + slider.height + 5, al_map_rgb(100, 100, 100));
}

bool is_mouse_over_slider(Slider slider, float mouse_x, float mouse_y) {
    return mouse_x >= slider.x && mouse_x <= slider.x + slider.width && mouse_y >= slider.y - slider.height && mouse_y <= slider.y;
}

int slider_clicked(all_sliders btn, float mouse_x, float mouse_y) {

    int output = 100;
    size_t length = sizeof(btn.all_sliders) / sizeof(btn.all_sliders[0]);
    for (int i = 0; i < length; i++)
    {
        if (mouse_x >= btn.all_sliders[i].x && mouse_x <= btn.all_sliders[i].x + btn.all_sliders[i].width && mouse_y >= btn.all_sliders[i].y - btn.all_sliders[i].height && mouse_y <= btn.all_sliders[i].y)
        {
            output = i;
        }
    }
    return output;
}
/*
int sgn(float i);
// standard sign function

float randf(float min, float max);
// generates a decimal random number between min and max

#define cart_lenght     90
#define cart_height     20
#define cart_Y_pose     400
#define wheel_diameter  10
#define wheel_distance  60
#define Mp_coeff        0.25
#define Lp_coeff        750
#define Mc_coeff        0.01
#define Mc_offset       20
#define wheel_offset    8
bool draw_cart(STATE state, DIM dim);
// draws the cart-pole
// with dimensions stated in dim,
// in the position determined by state.
// Input: state - state of the cart
//        dim   - phisical dimensions of the cart

#define g -9.80665    // (m/s/s) gravitational acceleration
bool compute_state(float force, STATE *s, DIM d, float dt);
// computes the new state of the system after a time dt
// Input: force - force applied on the cart
//        *s    - pointer to state structure
//        d     - phisical dimensions of the cart
//        dt    - integration step

#define base_width      0.9
#define max_delta_theta M_PI/2
bool is_state_unacceptable(STATE s, DIM d);
// checks if the current state is acceptable w.r.t. the defined boundaries
// Input:  s    - state structure
//         d    - phisical dimensions of the cart
// Output: 1    - state acceptable
//         0    - state NOT acceptable

void set_state(STATE *s, float x0, float v0, float theta0, float omega0);
// sets state s to desired values
// Input: *s     - pointer to state structure
//         x0     - desired cart position
//         v0     - desired cart speed
//         theta0 - desired pole angle
//         omega0 - desired pole angular velocity


int decode_x(float x, int* r);
// decodes the state of x
// Input: x - position of the cart
//        r - pointer to reward variable
// Output: state

int decode_v(float v, int* r);
// decodes the state of v
// Input: v - speed of the cart
//        r - pointer to reward variable
// Output: state

int decode_theta(float theta, int* r);
// decodes the state of theta
// Input: theta - angle of the pole
//        r     - pointer to reward variable
// Output: state

int decode_omega(float omega, int* r);
// decodes the state of omega
// Input: omega - angular velocity of the pole
//        r     - pointer to reward variable
// Output: state

int decode_state(STATE state, int* r);
// decodes the overall state
// Input: state - state of the cart-pole
//        r     - pointer to reward variable
// Output: state box number

void update_Q(float* Q, int box, int old_box, int action, int reward, float gamma, float alpha);
// Updates the Q matrix with the rewards values
// Input: Q       - pointer to the Q matrix (vector indiced by rows)
//        box     - actual box of the state
//        old_box - previous box of the state
//        action  - action that has been done
//        reward  - reward variable
//        gamma   - discount factor
//        alpha   - learning rate

int choose_action(float* Q, int old_box, float epsilon);
// Chooses the action for maximizing the reward or randomly, depending on epsilon
// Input:  Q         - pointer to the Q matrix (vector indiced by rows)
//         old_box   - previous state box
//         epsilon   - exploration factor
// Output: chosen action (-1 or +1, i.e. force from left or right)

void box_visualizer_2D(STATE s, const ALLEGRO_FONT *font);
// Shows the actual 4 state box in 2 2D graphs
// Input: s     - state of the cart-pole
//        font  - allegro font

bool manage_button(BUTTON* b, float x, float y, bool c, const ALLEGRO_FONT *font);
// Draws and manage the button b
// Input: b     - pointer to button structure
//        x     - mouse x
//        y     - mouse y
//        c     - mouse click
//        font  - allegro font
// Output: true if button released

float manage_slider(SLIDER* s, float x, float y, bool c, const ALLEGRO_FONT *font);
// Draws and manage the slider s
// Input: s     - pointer to slider structure
//        x     - mouse x
//        y     - mouse y
//        c     - mouse click
//        font  - allegro font
// Output: returns the value of the slider

void slider_constructor(SLIDER*s, float min, float max, int x0, int y0, int l, bool vertical, float value0, int bw, int bh, char* text);
// Constructs a slider s
// Input: s           - pointer to slider structure to be filled
//        min         - minimum value of the slider
//        max         - maximum value of the slider
//        x0          - starting x of the slider
//        y0          - starting y of the slider
//        l           - lenght of the slider in pixels
//        vertical    - true for vertical, false for horizontal
//        value0      - initial value of the slider
//        bw          - slider button width
//        bh          - slider button height


void graph_constructor(GRAPH*gr, float ymin, float ymax, int x0, int y0, int w, int h, int buff_size, char* text_x_axis, char* text_y_axis, char* text_title);
// Constructs a graph g
// Input: g           - pointer to graph structure to be filled
//        ymin        - minimum value of the graph on y axis
//        ymax        - maximum value of the graph on y axis
//        x0          - starting x of the graph
//        y0          - starting y of the graph
//        w           - width of the graph in pixels
//        h           - height of the graph in pixels
//        buff_size   - number of to be averaged to put one point in the graph
//        text_x_axis - label on x axis
//        text_y_axis - label on y axis
//        text_title  - graph title
*/

//Svoid button_constructor(BUTTON* b, int x0, int y0, int w, int h, char* text);
// Constructs a button b
// Input: b           - pointer to button structure to be filled
//        x0          - starting x of the button
//        y0          - starting y of the button
//        w           - width of the button in pixels
//        h           - height of the button in pixels
//        text        - label on the button

/*
float compute_pid(PID* p, STATE* s, float err);
// Computes the output of the PID controller
// Input: p   - pointer to PID structure
//        s   - pointer to STATE structure
//        err - error value
// Output: force to be applied to the cart

void plot_graph(GRAPH* gr, float v, const ALLEGRO_FONT *font, float dt, bool update);
// Draws and manage the graph g
// Input: g       - pointer to graph structure
//        v       - value to add to the graph
//        font    - allegro font
//        dt      - time increment between two subsequent points
//        update  - if true iserts the value v in the graph, if false not

float user_force(float x, float y, bool click, float noise_coeff, STATE state, DIM dim, const ALLEGRO_FONT *font);
// Computes and applies a user defined force to the cart by clicking and dragging with the mouse and thus producing an "explosion".
// Force is proportional to the dimension of the drawn circle and to the inverse of the distance to the cart.
// Also adds random noise due to "wind".
// Input: x           - mouse x
//        y           - mouse y
//        click       - mouse click
//        noise_coeff - moltiplicative coefficient for random noise force
//        state       - state of the cart-pole
//        dim         - phisical dimensions of the cart
//        font        - allegro font

bool save_to_file(char* file_name, float* Q, float* epsilon, float* epsilon_decay, float* gamma_, float* alpha, int* action_multiplier, float* noise_coeff, DIM* dim, PID* pid_theta, PID* pid_x, GRAPH* g_totRew, GRAPH* g_instRew, GRAPH* g_frame, GRAPH* g_tdErr);
// Saves all the input variables in a text file.
// Input: file_name   - name of the file where to save the variables. It must have .txt extension
//        "others"    - pointers to the variables to be saved
// Output: true if an error occurs

bool load_from_file(char* file_name, float* Q, float* epsilon, float* epsilon_decay, float* gamma_, float* alpha, int* action_multiplier, float* noise_coeff, DIM* dim, PID* pid_theta, PID* pid_x, GRAPH* g_totRew, GRAPH* g_instRew, GRAPH* g_frame, GRAPH* g_tdErr);
// Load to the input variables the values present in the specified text file.
// Input: file_name   - name of the file from where to load the variables. It must be an existing file with .txt extension produced by the function "save_to_file"
//        "others"    - pointers to the variables to be loaded
// Output: true if an error occurs

*/
#include "functions_videoTracking.c"
