#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>

/* FUNCTIONS DECLARATIONS */

float randf(float min, float max)
{
    //float x = (((float)rand()) / RAND_MAX) * (max - min) + min;
    float x = ((float)rand() / (float)RAND_MAX) * (max - min) + min; 
    return x;
}

int randi(int min, int max)
{
    //int x = ((double)rand() / RAND_MAX) * (max - min) + min;
    int x = rand() % (max - min + 1) + min;
    return x;
}

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


float euclidean_distance(float pos1[2], float pos2[2]) {
    float dx = pos2[0] - pos1[0];
    float dy = pos2[1] - pos1[1];
    return sqrt(dx * dx + dy * dy);
}

float* initialize_R() {
    float pos_box[2], pos_box_temp[2];
    float x, y;
    float distance;
    int state; 
    for (int i = 0; i < BXH; i++) {
        for (int j = 0; j < BXW; j++) {
            state = i * BXW + j;
            pos_box[0] = box_width / 2 + j * box_width;
            pos_box[1] = box_height / 2 + i * box_height;
            for (int action = 0; action < ACTION_NUM; action++)
            {
                x = pos_box[0];
                y = pos_box[1];

                switch (action)
                {
                case 0:
                    y = y + box_height;
                    break;
                case 1:
                    y = y - box_height;
                    break;
                case 2:
                    x = x + box_width;
                    break;
                case 3:
                    x = x - box_width;
                    break;
                case 4:
                    break;
                }
                // 0 sotto
                // 1 sopra 
                // 2 destra 
                // 3 sinistra
                // 4 resta fermo
                pos_box_temp[0] = x;
                pos_box_temp[1] = y;

                if (x < box_width * BXW && x > 0 && y < box_height * BXH && y > 0) {
                   distance = euclidean_distance(pos_box_temp, pos_center);
                   if (distance < 2*box_width) {
                       if (distance <= box_width/2)
                           R[state * ACTION_NUM + action] = REWARD_POSITIVE;
                       else 
                            R[state* ACTION_NUM + action] = REWARD_NEUTRAL;
                   }
                   else {
                       R[state * ACTION_NUM + action] = (distance/100)*REWARD_NEGATIVE;
                   }
                }
                else {
                    R[state * ACTION_NUM + action] = REWARD_NEGATIVE * 5;
                }
            }
        }
    }
    return R;
}

void initialize_Q() {
    for (int i = 0; i < BXW * BXH; i++)
    {
        for (int j = 0; j < ACTION_NUM; j++)
        {
            Q[i * ACTION_NUM + j] = 0.0;
        }
    }
}

int choose_action(float* Q, int state, float epsilon)
{
    // actions from 0 to 3
    int max_index = 0;
    if (randf(0, 1) <= epsilon)
    {
        max_index = randi(0, 3);
    }
    else
    {
        for (int act = 1; act < ACTION_NUM; act++)
        {
            if (Q[state * ACTION_NUM + act] > Q[state * ACTION_NUM + max_index])
                max_index = act;
        }
    }

    return max_index;
}

float compute_maxQ(float* Q, int state) {
    int maxQ, max_index = 0;
    for (int act = 1; act < ACTION_NUM; act++)
    {
        if (Q[state * ACTION_NUM + act] > Q[state * ACTION_NUM + max_index])
            max_index = act;
    }
    return  Q[state * ACTION_NUM + max_index];
}

int compute_state(int action, int current_state) {
    int next_state;
    switch (action)
    {
    case 0:
        next_state = current_state + BXW;
        break;
    case 1:
        next_state = current_state - BXW;
        break;
    case 2:
        if ((current_state + 1) % BXW == 0)
            next_state = -100;
        else next_state = current_state + 1; 
        break;
    case 3:
        if (current_state % BXW == 0)
            next_state = -100;
        else next_state = current_state - 1;         
        break;
    case 4: 
        next_state = current_state; 
        break; 
    }

    return next_state;
}

float decode_reward(int action, int state) {
    return R[state * ACTION_NUM + action];
}
    
bool isdone(int next_state, int count)
{
    bool done = false;
    if (next_state >= BXW * BXH || next_state < 0)
        done = true;
    
    return done;
}



void ball_center_coordinates(ALLEGRO_DISPLAY* display, float* ball_center) {
    // Riconoscimento pixel rossi
    ALLEGRO_BITMAP* backbuffer;
    ALLEGRO_COLOR pixel_color;
    unsigned char r, g, b;
    float x_center = 0, y_center = 0;
    int counter = 0;
//    float* ball_center[2];

    backbuffer = al_get_backbuffer(display);
    al_lock_bitmap(backbuffer, ALLEGRO_PIXEL_FORMAT_ANY, ALLEGRO_LOCK_READONLY);
    // 1, 1, disp.w_display / 2 - disp.edge, disp.h_display / 2
    for (int x = 1; x < disp.w_display / 2 - disp.edge; x++) {
        for (int y = 1; y < disp.h_display / 2; y++) {
            pixel_color = al_get_pixel(backbuffer, x, y);
            al_unmap_rgb(pixel_color, &r, &g, &b);
            if (r == 255 && g == 0 && b == 0) {
                x_center = x_center + x;
                y_center = y_center + y;
                counter = counter + 1;
            }
        }
    }

    if (counter > 0) {
        ball_center[0] = x_center / counter;
        ball_center[1] = y_center / counter;
    }
    else {
        ball_center[0] = ball_center[1] = -100;  // Valori di default se non si trovano pixel rossi
    }

    al_unlock_bitmap(backbuffer);

}


int compute_ball_state(float* ball_center) {
    int state; 
    if (ball_center[0] == -100)
    {
        state = -1; 
    }
    else 
    {
        int x_box = int(ball_center[0]/ box_width);
        int y_box = int(ball_center[1] / box_height);
        state = y_box * BXW + x_box;
    }

    return state;
}

void draw_trajectory(int trajectory_ID) {
    al_draw_line(trajectory_ax_x.x_1, trajectory_ax_x.y_1, trajectory_ax_x.x_2, trajectory_ax_x.y_2, al_map_rgb(255, 255, 255), 2);
    al_draw_line(trajectory_ax_y.x_1, trajectory_ax_y.y_1, trajectory_ax_y.x_2, trajectory_ax_y.y_2, al_map_rgb(255, 255, 255), 2);

    switch (trajectory_ID)
    {
    case 0:
        for (int x = 0; x < (trajectory_ax_x.x_2 - trajectory_ax_x.x_1); x++) {
            y_plot[x] = amplitude * sin(frequency * x + phase) + trajectory_ax_y.y_1 + (trajectory_ax_y.y_2 - trajectory_ax_y.y_1) / 2;
            x_plot[x] = x + trajectory_ax_x.x_1;
            al_draw_pixel(x_plot[x], y_plot[x], al_map_rgb(255, 0, 0)); // Disegna un pixel rosso
        }
        break;
    }
}

void draw_results() {
    al_draw_line(ax_totreward_x.x_1, ax_totreward_x.y_1, ax_totreward_x.x_2, ax_totreward_x.y_2, al_map_rgb(255, 255, 255), 2); 
    al_draw_line(ax_totreward_y.x_1, ax_totreward_y.y_1, ax_totreward_y.x_2, ax_totreward_y.y_2, al_map_rgb(255, 255, 255), 2); 
    //al_draw_line(ax_framecount_y.x_1, ax_framecount_y.y_1, ax_framecount_y.x_2, ax_framecount_y.y_2, al_map_rgb(255, 255, 255), 2);

    al_draw_line(ax_framecount_x.x_1, ax_framecount_x.y_1, ax_framecount_x.x_2, ax_framecount_x.y_2, al_map_rgb(255, 255, 255), 2); 
    al_draw_line(ax_framecount_y.x_1, ax_framecount_y.y_1, ax_framecount_y.x_2, ax_framecount_y.y_2, al_map_rgb(255, 255, 255), 2);

}



void visualize(float camera_pan, float camera_tilt) {

    al_draw_filled_rectangle(0, 0, disp.w_display, disp.h_display / 2 + 30, al_map_rgb(0, 0, 0)); // rectangle used to cover the red ball
    draw_trajectory(trajectory_ID); // draw overall trajectory and axes
    al_draw_filled_circle(ball_state.x_2, ball_state.y_2, 5, al_map_rgb(0, 255, 0));

    // zoom
    al_draw_filled_rectangle(1, 1, disp.w_display / 2 - disp.edge, disp.h_display / 2, al_map_rgb(255, 255, 255));
    if ((ball_state.x_1 < disp.w_display / 2 - disp.edge) && (ball_state.y_1 < disp.h_display / 2)) {
        al_draw_filled_circle(ball_state.x_1, ball_state.y_1, 20, al_map_rgb(255, 0, 0));
    }

    al_flip_display();

}

void move_camera(int action) {
    switch (action)
    {
    case 0:
        camera_tilt = box_height;
        break;
    case 1:
        camera_tilt = -box_height;
        break;
    case 2:
        camera_pan = box_width;
        break;
    case 3:
        camera_pan = -box_width;
        break;
    case 4:
        break;
    }
}

float rescale(char axis, AXES axes, int min, int max, float value)
{
    int range = max - min; 
    int pixel; 
    if (axis == 'x')
        pixel = (value - min) * (axes.x_2 - axes.x_1) /range  + axes.x_1; 
    else if (axis == 'y')
        pixel = axes.y_2 - (value - min) * (axes.y_2 - axes.y_1) / range;
    return pixel; 
}



void start_training(ALLEGRO_DISPLAY* display) {
    // Initialization
    float* R_prova = initialize_R();
    printf("\nPoints Matrix : \n");

    for (int st = 0; st < BXW * BXH; st++)
    {
        for (int act = 0; act < 5; act++)
        {
            printf("%f\t", R_prova[st * 5 + act]);
        }
        printf("\n");
    }
    printf("\n\n\n");

    initialize_Q();
    float epsilon = 1;
    float gamma = 0.9;
    float alpha = 0.1;
    int state;
    int action = -100, next_state;
    bool is_acceptable;
    float reward, maxQ;
    int episod = 0;
    float ball_center[2];

    float pixel_x_plot1;
    float pixel_y_plot1; 
    float pixel_x_tmp_plot1;
    float pixel_y_tmp_plot1;
    float pixel_x_plot2;
    float pixel_y_plot2;
    float pixel_x_tmp_plot2 = 0;
    float pixel_y_tmp_plot2 = 0;

    draw_results();
    float count_step[2500];
    for (int episode = 0; episode < episode_target; episode++) {
        tot_reward = 0; 
        bool done = false;
        int steps = 0;
        int counter = 0;
        int traj_counter = 0; 
 
        while (!done && steps < steps_target)
        {
            camera_pan = 0.0; 
            camera_tilt = 0.0; 
            move_camera(action); 
            if (steps == 0)
            {
                ball_state.x_1 = (disp.w_display - disp.edge) / 4 - disp.w_display/8; // disp.w_display/8 è un offset per far iniziare il training non dal centro
                ball_state.y_1 = disp.h_display / 4;
            }
            else
            {
                //ball_state.x_1 = ball_state.x_1 + (x_plot[traj_counter] - x_plot[traj_counter - 1])/2000 + camera_pan; // 20 è una scale_factor altrimenti la pallina va troppo veloce
                //ball_state.y_1 = ball_state.y_1 + (y_plot[traj_counter] - y_plot[traj_counter - 1])/2000 + camera_tilt; // 20 è una scale_factor altrimenti la pallina va troppo veloce
                ball_state.x_1 = ball_state.x_1 + camera_pan;
                ball_state.y_1 = ball_state.y_1 + camera_tilt;
            }
                
            ball_state.x_2 = x_plot[traj_counter];
            ball_state.y_2 = y_plot[traj_counter]; 
            visualize(camera_pan, camera_tilt);
            ball_center_coordinates(display, ball_center);
            state = compute_ball_state(ball_center);
            action = choose_action(Q, state, epsilon);
            next_state = compute_state(action, state);
            // scelta reward da R
            reward = decode_reward(action, state);
            tot_reward += reward; 
            // compute new Q target
            maxQ = compute_maxQ(Q, state);
            if (counter == 5)
            {
                reward = REWARD_POSITIVE; 
            }
            // update Q
            Q[state * ACTION_NUM + action] = (1 - alpha) * Q[state * ACTION_NUM + action] + alpha * (reward + gamma * maxQ);
            
            printf("%f \n", epsilon);

            //printf("\Q Matrix : \n");

            /*for (int st = 0; st < BXW * BXH; st++)
            {
                for (int act = 0; act < 4; act++)
                {
                    printf("%f\t", Q[st * 4 + act]);
                }
                printf("\n");
            }
            printf("\n\n\n");*/

            done = isdone(next_state, counter);           

            if (reward == REWARD_NEUTRAL || reward == REWARD_POSITIVE)
                counter++;



            traj_counter++;
            steps++;
        }

        count_step[episode] = steps; 

        epsilon = 0.999*epsilon;
        //printf("%f", epsilon);

        pixel_x_plot1 = rescale('x', ax_totreward_x, 0, episode_target, episode);
        pixel_y_plot1 = rescale('y', ax_totreward_y, -10000, 5000, tot_reward);

        pixel_x_plot2 = rescale('x', ax_framecount_x, 0, episode_target, episode);
        pixel_y_plot2 = rescale('y', ax_framecount_y, 0, steps_target, steps);
        printf("tot reward %f \n", tot_reward);


        if(episode > 0)
        {
            al_draw_line(pixel_x_tmp_plot1, pixel_y_tmp_plot1, pixel_x_plot1, pixel_y_plot1, al_map_rgb(255, 0, 0), 0.5);
            al_draw_line(pixel_x_tmp_plot2, pixel_y_tmp_plot2, pixel_x_plot2, pixel_y_plot2, al_map_rgb(255, 0, 0), 0.5);
        }
        pixel_x_tmp_plot1 = pixel_x_plot1;
        pixel_y_tmp_plot1 = pixel_y_plot1;
        pixel_x_tmp_plot2 = pixel_x_plot2;
        pixel_y_tmp_plot2 = pixel_y_plot2;
        //al_draw_pixel(ax_totreward_x.x_1 + episode, ax_totreward_x.y_1 + tot_reward/10000, al_map_rgb(0, 0, 255));
        //al_draw_pixel(ax_framecount_x.x_1 + episode, ax_framecount_x.y_1 + steps, al_map_rgb(0, 0, 255));
        al_flip_display(); 
    }
    
    int daje = 0; 
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
