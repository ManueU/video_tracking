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


// Mapping Action to an Action ID (0-26)
int get_action_id(Action action) {
    return (action.pan + 1) * (NUM_PAN + NUM_TILT + NUM_VEL) + (action.tilt + 1) * NUM_PAN + (action.velocity - 1);
}

// Mapping Action ID to Action
Action get_action_from_id(int action_id) {
    Action action;
    action.pan = (action_id / (NUM_PAN + NUM_TILT + NUM_VEL)) - 1;
    action.tilt = ((action_id % (NUM_PAN + NUM_TILT + NUM_VEL)) / NUM_PAN) - 1;
    action.velocity = (action_id % NUM_VEL) + 1;
    return action;
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
            for (int action_id = 0; action_id < ACTION_NUM; action_id++)
            {
                action = get_action_from_id(action_id);
                x = pos_box[0] + action.pan * box_width * action.velocity;
                y = pos_box[1] - action.tilt * box_width * action.velocity;

                pos_box_temp[0] = x;
                pos_box_temp[1] = y;

                if (x < box_width * BXW && x > 0 && y < box_height * BXH && y > 0) {
                   distance = euclidean_distance(pos_box_temp, pos_center);
                   if (distance < 2*box_width) {
                       if (distance <= box_width/2)
                           R[state * ACTION_NUM + action_id] = REWARD_POSITIVE;
                       else 
                            R[state* ACTION_NUM + action_id] = REWARD_NEUTRAL;
                   }
                   else {
                       R[state * ACTION_NUM + action_id] = (distance/100)*REWARD_NEGATIVE;
                   }
                }
                else {
                    R[state * ACTION_NUM + action_id] = REWARD_NEGATIVE * 8;
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
    int max_index = 0;
    if (randf(0, 1) <= epsilon)
    {
        max_index = randi(0, ACTION_NUM - 1);
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

int compute_ball_state(float* ball_center) {
    int state;
    if (ball_center[0] == -100)
    {
        state = -1;
    }
    else
    {
        int x_box = int(ball_center[0] / box_width);
        int y_box = int(ball_center[1] / box_height);
        state = y_box * BXW + x_box;
    }

    return state;
}

int compute_state(int current_state, float* ball_center) {
    int next_state;
    float ball_center_next[2]; 

    ball_center_next[0] = ball_center[0] + action.pan * box_width * action.velocity;
    ball_center_next[1] = ball_center[1]-action.tilt * box_height * action.velocity;
    next_state = compute_ball_state(ball_center_next);

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

void move_camera(int action_id) {
    action = get_action_from_id(action_id);
    camera_pan = action.pan * box_width * action.velocity;
    camera_tilt = - action.tilt * box_width * action.velocity;
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

void load_Q(char* file_path) {
    FILE* file;
    errno_t err;  // Variable to store error codes for fopen_s

    err = fopen_s(&file, file_path, "r");

    if (err != 0) {
        printf("Error: Could not open the file.\n");
    }

    float number;  // Define buffer size large enough to hold a word
    int count = 0; 
    while (fscanf_s(file, "%f", &number) != EOF) {
        if (count == 0)
            gamma = number;
        else if (count == 1)
            alpha = number; 
        else 
            Q[count - 2] = number; 
        count++; 
    }

    fclose(file);
}

void start_simulation(ALLEGRO_DISPLAY* display) {
    char filename[] = "C:/Users/manue/OneDrive - Scuola Superiore Sant'Anna/Documents/PhD - Manuela Uliano/04_Courses/03_Neural Network and Deep Learning/2_Project/video_tracking/Q_matrix_array.txt"; 
    load_Q(filename); 
     


}

void start_training(ALLEGRO_DISPLAY* display) {
    // Initialization
    float* R_prova = initialize_R();
    printf("\nR Matrix : \n");

    for (int st = 0; st < BXW * BXH; st++)
    {
        for (int act = 0; act < ACTION_NUM; act++)
        {
            printf("%f\t", R_prova[st * ACTION_NUM + act]);
        }
        printf("\n");
    }
    printf("\n\n\n");

    /*action = get_action_from_id(3);
    printf("pan %d \n", action.pan); 
    printf("tilt %d \n", action.tilt);
    printf("vel %d \n", action.velocity);*/



    initialize_Q();
    float epsilon = 0.1;
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
        int previous_step = 0; 
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
                //ball_state.x_1 = ball_state.x_1 + (x_plot[traj_counter] - x_plot[traj_counter - 1]) + camera_pan; // 20 è una scale_factor altrimenti la pallina va troppo veloce
                //ball_state.y_1 = ball_state.y_1 + (y_plot[traj_counter] - y_plot[traj_counter - 1]) + camera_tilt; // 20 è una scale_factor altrimenti la pallina va troppo veloce
                ball_state.x_1 = ball_state.x_1 + camera_pan;
                ball_state.y_1 = ball_state.y_1 + camera_tilt;
            }
                
            ball_state.x_2 = x_plot[traj_counter];
            ball_state.y_2 = y_plot[traj_counter]; 
            visualize(camera_pan, camera_tilt);
            ball_center_coordinates(display, ball_center);
            state = compute_ball_state(ball_center);
            action = choose_action(Q, state, epsilon);
            next_state = compute_state(state, ball_center);
            // scelta reward da R
            reward = decode_reward(action, state);
            tot_reward += reward; 
            // compute new Q target
            maxQ = compute_maxQ(Q, state);
            if (counter == 3)
            {
                reward = REWARD_POSITIVE * 2;
                counter = 0; 
            }
            // update Q
            Q[state * ACTION_NUM + action] = (1 - alpha) * Q[state * ACTION_NUM + action] + alpha * (reward + gamma * maxQ);
            
            printf("%f \n", epsilon);

            printf("\Q Matrix : \n");

            for (int st = 0; st < BXW * BXH; st++)
            {
                for (int act = 0; act < ACTION_NUM; act++)
                {
                    printf("%f\t", Q[st * ACTION_NUM + act]);
                }
                printf("\n");
            }
            printf("\n\n\n");

            done = isdone(next_state, counter);           

            if (counter == 0)
                previous_step = steps - 1;

            if (reward == REWARD_NEUTRAL || reward == REWARD_POSITIVE)
            {
                if ((steps - previous_step) == 1)
                {
                    counter++; 
                    previous_step = steps; 
                }
            }
            else
                counter = 0; 

            
            traj_counter++;
            steps++;
        }

        count_step[episode] = steps; 

        epsilon = 0.997*epsilon;
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


#include "functions_videoTracking.c"
