// ProvaC.cpp : Questo file contiene la funzione 'main', in cui inizia e termina l'esecuzione del programma.
//

#include <allegro5/allegro5.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>
//#include "allegro5/allegro_image.h"
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_color.h>
#include <math.h>
#include "variables_videoTracking.h"
#include "functions_videoTracking.h"
#include <stdio.h>
#include <stdlib.h>

int main()
{    
    al_init();
    al_init_font_addon();
    al_init_ttf_addon();
    al_install_mouse();
    al_install_keyboard();
    al_init_primitives_addon();

    ALLEGRO_EVENT_QUEUE* event_queue = al_create_event_queue();
    ALLEGRO_TIMER* timer = al_create_timer(1.0 / 60);
    ALLEGRO_FONT* font = al_load_ttf_font("arial.ttf", 24, 0);
    ALLEGRO_DISPLAY* display;
    display =  al_create_display(disp.w_display, disp.h_display);
    al_register_event_source(event_queue, al_get_display_event_source(display));
    al_register_event_source(event_queue, al_get_timer_event_source(timer));
    al_register_event_source(event_queue, al_get_mouse_event_source());

    int camera_pan = 0, camera_tilt = 0;
    ALL_BUTTONS buttons;
    buttons.all_buttons[0] = load;
    buttons.all_buttons[1] = save;
    buttons.all_buttons[2] = start_simulation;
    buttons.all_buttons[3] = start_training;

    all_sliders sliders; 
    sliders.all_sliders[0] = epsilon; 
    sliders.all_sliders[1] = gamma; 
    sliders.all_sliders[2] = alpha;

    al_start_timer(timer);
    double color_change_timer = 0.0;
    const double color_change_duration = 0.5;

    // Esempio di procedimento
    float* R_prova = initialize_R();
    initialize_Q();
    float epsilon = 0.7;
    int state = 8;
    int action = choose_action(Q, state, epsilon);
    int next_state = compute_state(action, state);
    // scelta reward da R
    // compute new Q target
    // update Q

    printf("\nPoints Matrix : \n");
    for (int state = 0; state < BXW * BXH; state++)
    {
        for (int action = 0; action < 4; action++)
        {
            printf("%f\t", R_prova[state * 4 + action]);
        }
        printf("\n");
    }
    printf("\n\n\n");


    int button_index;
    int slider_index;
    bool running = true;
    bool flag = false; 
    while (running) {
        ALLEGRO_EVENT ev;
        al_wait_for_event(event_queue, &ev);

        if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            running = false;
        }
        else if (ev.type == ALLEGRO_EVENT_MOUSE_BUTTON_DOWN) {
            slider_index = slider_clicked(sliders, ev.mouse.x, ev.mouse.y);
                if (slider_index != 100) {                    
                    sliders.all_sliders[slider_index].value = (ev.mouse.x - sliders.all_sliders[slider_index].x) / sliders.all_sliders[slider_index].width;
                    if (sliders.all_sliders[slider_index].value < 0) sliders.all_sliders[slider_index].value = 0;
                    if (sliders.all_sliders[slider_index].value > 1) sliders.all_sliders[slider_index].value = 1;
            } else 
            { 
                button_index = button_clicked(buttons, ev.mouse.x, ev.mouse.y);
                if(button_index != 100)
                {
                    flag = true;
                    buttons.all_buttons[button_index].color = al_map_rgb(255, 0, 0);
                    color_change_timer = al_get_time();
                    // funzione che fa fare cose ai bottoni
                }
            }
        }

        else if (ev.type == ALLEGRO_EVENT_TIMER) {

            al_clear_to_color(al_map_rgb(0, 0, 0));
            draw_button(buttons.all_buttons[0], font, buttons.all_buttons[0].color);
            draw_button(buttons.all_buttons[1], font, buttons.all_buttons[1].color);
            draw_button(buttons.all_buttons[2], font, buttons.all_buttons[2].color);
            draw_button(buttons.all_buttons[3], font, buttons.all_buttons[3].color);
            draw_slider(sliders.all_sliders[0]);
            draw_slider(sliders.all_sliders[1]);
            draw_slider(sliders.all_sliders[2]);


            if (flag) {
                double elapsed_time = al_get_time() - color_change_timer;
                if (elapsed_time >= color_change_duration) {
                    buttons.all_buttons[button_index].color = al_map_rgb(0,0,255);
                    flag = false;
                }
            }

            state.x_c2 = state.x_c2 + 1;
            if (state.x_c2 > disp.w_display) { state.x_c2 = disp.w_display / 2 + disp.edge; }
            state.y_c2 = sin(0.1 * state.x_c2) * disp.h_display / 4 + disp.h_display / 4;
            state.x_c1 = 2 * state.x_c2 - (disp.w_display / 2 - disp.edge) * 2 - (disp.w_display / 2 - disp.edge) / 2;
            state.y_c1 = 2 * state.y_c2 - disp.h_display / 4;


            al_draw_filled_rectangle(1, 1, disp.w_display / 2 - disp.edge, disp.h_display / 2, al_map_rgb(255, 255, 255));
            if ((state.x_c1 - camera_pan < disp.w_display / 2 - disp.edge) && (state.y_c1 - camera_tilt < disp.h_display / 2)) {
                al_draw_filled_circle(state.x_c1 - camera_pan, state.y_c1 - camera_tilt, 20, al_map_rgb(255, 0, 0));
            }
            al_draw_filled_rectangle(disp.w_display / 2 + disp.edge, 1, disp.w_display, disp.h_display / 2, al_map_rgb(255, 255, 255));
            al_draw_filled_circle(state.x_c2, state.y_c2, 5, al_map_rgb(255, 0, 0));
            //al_draw_filled_circle(420, 300, 10, al_map_rgb(255, 0, 0));

            al_flip_display();
        }

    }
    al_destroy_display(display);
    


}

// Per eseguire il programma: CTRL+F5 oppure Debug > Avvia senza eseguire debug
// Per eseguire il debug del programma: F5 oppure Debug > Avvia debug

// Suggerimenti per iniziare: 
//   1. Usare la finestra Esplora soluzioni per aggiungere/gestire i file
//   2. Usare la finestra Team Explorer per connettersi al controllo del codice sorgente
//   3. Usare la finestra di output per visualizzare l'output di compilazione e altri messaggi
//   4. Usare la finestra Elenco errori per visualizzare gli errori
//   5. Passare a Progetto > Aggiungi nuovo elemento per creare nuovi file di codice oppure a Progetto > Aggiungi elemento esistente per aggiungere file di codice esistenti al progetto
//   6. Per aprire di nuovo questo progetto in futuro, passare a File > Apri > Progetto e selezionare il file con estensione sln
