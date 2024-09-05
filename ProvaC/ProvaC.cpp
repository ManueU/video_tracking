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

    ALL_BUTTONS buttons;
    buttons.all_buttons[0] = load;
    buttons.all_buttons[1] = save;
    buttons.all_buttons[2] = button_start_simulation;
    buttons.all_buttons[3] = button_start_training;

    all_sliders sliders; 
    sliders.all_sliders[0] = epsilon_slider; 
    sliders.all_sliders[1] = gamma_slider;
    sliders.all_sliders[2] = alpha_slider;

    al_start_timer(timer);
    double color_change_timer = 0.0;
    const double color_change_duration = 0.5;
    srand(static_cast<unsigned int>(time(0)));  // Initialize seed randomly


    /*printf("\nPoints Matrix : \n");

    for (int st = 0; st < BXW * BXH; st++)
    {
        for (int act = 0; act < 4; act++)
        {
            printf("%f\t", R_prova[st * 4 + act]);
        }
        printf("\n");
    }
    printf("\n\n\n");

    printf("\State: \n");
    printf("%d\t", state);
    printf("\n \Action: \n");
    printf("%d\t", action);
    printf("\n \Next state: \n");
    printf("%d\t", next_state);*/

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

            // change color when pressed
            if (flag) {
                double elapsed_time = al_get_time() - color_change_timer;
                if (elapsed_time >= color_change_duration) {
                    buttons.all_buttons[button_index].color = al_map_rgb(0,0,255);
                    flag = false;
                    switch (button_index)
                    {
                    case 3:
                        start_training(display);
                        break;
                    }
                }
            }

           al_flip_display();
        }
        
        //start_training(display);

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
