
/*
int sgn(float i)
{
  if (i < 0) return -1;
  if (i > 0) return 1;
  return 0;
}

float randf(float min, float max)
{
  float x = (((float)rand())/RAND_MAX)*(max-min) + min;
  return x;
}

bool draw_cart(STATE state, DIM dim)
{
  static float x_poleTip;
  static float y_poleTip;
  static float cart_center;
  cart_center = (state.x+1)*dW/2;
  x_poleTip = dim.Lp*Lp_coeff * sin(state.theta) + cart_center;
  y_poleTip = -(dim.Lp*Lp_coeff * cos(state.theta)) + cart_Y_pose-(cart_height*dim.Mc*Mc_coeff);

  al_draw_line(cart_center, cart_Y_pose-(cart_height*dim.Mc*Mc_coeff), x_poleTip, y_poleTip, al_map_rgb(0,0,0), 6);  // pole
  al_draw_filled_circle(x_poleTip, y_poleTip, dim.Mp*Mp_coeff, al_map_rgb(0,0,0));  // weight
  al_draw_filled_circle(cart_center, cart_Y_pose-(cart_height*dim.Mc*Mc_coeff), 8, al_map_rgb(180,0,0));  //pole base
  al_draw_filled_rectangle(cart_center-(cart_lenght*dim.Mc*Mc_coeff+Mc_offset)/2,cart_Y_pose-(cart_height*dim.Mc*Mc_coeff),cart_center+(cart_lenght*dim.Mc*Mc_coeff+Mc_offset)/2, cart_Y_pose, al_map_rgb(180,0,0));  // cart body
  al_draw_filled_circle(cart_center-(wheel_distance*dim.Mc*Mc_coeff)/2, cart_Y_pose, (wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset, al_map_rgb(15,15,15));  //wheel
  al_draw_filled_circle(cart_center+(wheel_distance*dim.Mc*Mc_coeff)/2, cart_Y_pose, (wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset, al_map_rgb(15,15,15));  //wheel
  al_draw_line(0, cart_Y_pose+(wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset, 2000, cart_Y_pose+(wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset, al_map_rgb(180,0,0), 5);  // red floor
  al_draw_line(dW/2-base_width*dW/2, cart_Y_pose+(wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset, dW/2+base_width*dW/2, cart_Y_pose+(wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset, al_map_rgb(0,140,0), 5);  // green floor
  return true;
}

bool compute_state(float force, STATE *s, DIM d, float dt)
{
  static float sinT, cosT;  // sin theta, cos decode_theta
  static float a, alpha;  // linear & angular acceleration

  sinT = -sin(s->theta);
  cosT = cos(s->theta);

  alpha = ((d.Mc+d.Mp)*g*sinT - (force+d.Mp*d.Lp*pow(s->omega,2)*sinT)*cosT) / (4/3*(d.Mc+d.Mp)*d.Lp-d.Mp*d.Lp*pow(cosT,2));
  a = (force+d.Mp*d.Lp*(pow(s->omega,2)*sinT-alpha*cosT)) / (d.Mc+d.Mp);

  s->x += s->v*dt;
  s->v += a*dt;
  s->theta += s->omega*dt;
  s->omega += alpha*dt;

  return true;
}

bool is_state_unacceptable(STATE s, DIM d)
{
  if ((s.x < -base_width) || (s.x > base_width))
    return true;
  if (s.theta < (-max_delta_theta) || s.theta > (max_delta_theta/2))
    return true;
  return false;
}

void set_state(STATE *s, float x0, float v0, float theta0, float omega0)
{
  s->x = x0;
  s->v = v0;
  s->theta = theta0;
  s->omega = omega0;
}

int decode_x(float x, int* r)
{
  if (x < -XL)
  {
    *r += RXL;
    return 0;
  }
  if (x <  XL)
  {
    *r += RXP;
    return 1;
  }
  *r += RXL;
  return 2;
}

int decode_v(float v, int* r)
{
  if (v < -VL)
  {
    *r += RVL;
    return 0;
  }
  if (v <  VL)
  {
    *r += RVP;
    return 1;
  }
  *r += RVL;
  return 2;
}

int decode_theta(float theta, int* r)
{
  if (theta < -T6)
  {
    *r += RT6;
    return 0;
  }
  if (theta < -T1)
  {
    *r += RT1;
    return 1;
  }
  if (theta <  0)
  {
    *r += RT0;
    return 2;
  }
  if (theta <  T1)
  {
    *r += RT0;
    return 3;
  }
  if (theta <  T6)
  {
    *r += RT1;
    return 4;
  }
  *r += RT6;
  return 5;
}

int decode_omega(float omega, int* r)
{
  if (omega < -W50)
  {
    *r += RW50;
    return 0;
  }
  if (omega <  W50)
  {
    *r += RW0;
    return 1;
  }
  *r += RW50;
  return 2;
}

int decode_state(STATE state, int* r)
{
  int box;
  box = decode_x(state.x, r);
  box += decode_v(state.v, r)*NBX;
  box += decode_theta(state.theta, r)*NBX*NBV;
  box += decode_omega(state.omega, r)*NBX*NBV*NBT;
  return box;
}

void update_Q(float* Q, int box, int old_box, int action, int reward, float gamma, float alpha)
{
  // get the index for the column for the action
  if(action < 0)
    action = 0;
  else
    action = 1;

  // get the max in the (new) box row
  static int row_max;// find the
  if(Q[(2*box)+0] > Q[(2*box)+1])
    row_max = Q[(2*box)+0];
  else
    row_max = Q[(2*box)+1];

  // compute Q
  Q[(2*old_box)+action] = (1-alpha)*Q[(2*old_box)+action] + alpha*(reward+gamma*(row_max));
}

int choose_action(float* Q, int box, float epsilon)
{
  if(randf(0,1) <= epsilon)
  {
    if(randf(0,1) < 0.5)
      return -1;
    else
      return 1;
  }
  else
  {
    if(Q[(2*box)+0] > Q[(2*box)+1])
      return -1;
    else
      return 1;
  }
}

#define box_X1 45
#define box_Y1 470
#define box_X2 (box_X1+160)
#define box_Y2 470
#define box_vis_W1 30*3
#define box_vis_H1 30*3
#define box_vis_W2 30*6
#define box_vis_H2 30*3
#define text_bias 2
#define lines_w 1
#define border_w 2
void box_visualizer_2D(STATE s, const ALLEGRO_FONT *font)
{
  int dummy;

  if(DEBUG)
  {
    al_draw_filled_circle(box_X2, box_Y2, 3, al_map_rgb(255,0,0));  // For positioning
    al_draw_filled_circle(box_X1, box_Y1, 3, al_map_rgb(255,0,0));  // For positioning
  }

  // X - V BOXES
  // find the box
  int x_state = decode_x(s.x, &dummy);
  int v_state = decode_v(s.v, &dummy);
  // draw the box
  al_draw_filled_rectangle(box_X1+x_state*round(box_vis_W1/3), box_Y1+(2-v_state)*round(box_vis_H1/3), box_X1+(x_state+1)*round(box_vis_W1/3), box_Y1+(3-v_state)*round(box_vis_H1/3), al_map_rgb(180,0,0));

  // external borders
  al_draw_rectangle(box_X1, box_Y1, box_X1+box_vis_W1, box_Y1+box_vis_H1, al_map_rgb(0,0,0), border_w);
  //horizontal lines
  al_draw_line(box_X1, box_Y1+round(box_vis_H1*1/3), box_X1+box_vis_W1, box_Y1+round(box_vis_H1*1/3), al_map_rgb(0,0,0), lines_w);
  al_draw_line(box_X1, box_Y1+round(box_vis_H1*2/3), box_X1+box_vis_W1, box_Y1+round(box_vis_H1*2/3), al_map_rgb(0,0,0), lines_w);
  al_draw_text(font, al_map_rgb(0,0,0), box_X1+round(box_vis_W1*1/3), box_Y1+box_vis_H1+text_bias, ALLEGRO_ALIGN_CENTRE, "-XL");
  al_draw_text(font, al_map_rgb(0,0,0), box_X1+round(box_vis_W1*2/3), box_Y1+box_vis_H1+text_bias, ALLEGRO_ALIGN_CENTRE, "+XL");
  al_draw_text(font, al_map_rgb(0,0,0),  box_X1-text_bias, box_Y1+round(box_vis_H1*2/3)-FONTHEIGHT, ALLEGRO_ALIGN_RIGHT, "-VL");
  al_draw_text(font, al_map_rgb(0,0,0),  box_X1-text_bias, box_Y1+round(box_vis_H1*1/3)-FONTHEIGHT, ALLEGRO_ALIGN_RIGHT, "+VL");

  // vertical lines
  al_draw_line(box_X1+round(box_vis_W1*1/3), box_Y1, box_X1+round(box_vis_W1*1/3), box_Y1+box_vis_H1, al_map_rgb(0,0,0), lines_w);
  al_draw_line(box_X1+round(box_vis_W1*2/3), box_Y1, box_X1+round(box_vis_W1*2/3), box_Y1+box_vis_H1, al_map_rgb(0,0,0), lines_w);


  // theta - omega BOXES
  // find the box
  int theta_state = decode_theta(s.theta, &dummy);
  int omega_state = decode_omega(s.omega, &dummy);
  // draw the box
  al_draw_filled_rectangle(box_X2+theta_state*round(box_vis_W2/6), box_Y2+(2-omega_state)*round(box_vis_H2/3), box_X2+(theta_state+1)*round(box_vis_W2/6), box_Y2+(3-omega_state)*round(box_vis_H2/3), al_map_rgb(180,0,0));

  // external borders
  al_draw_rectangle(box_X2, box_Y2, box_X2+box_vis_W2, box_Y2+box_vis_H2, al_map_rgb(0,0,0), border_w);
  //horizontal lines
  al_draw_line(box_X2, box_Y2+round(box_vis_H2*1/3), box_X2+box_vis_W2, box_Y2+round(box_vis_H2*1/3), al_map_rgb(0,0,0), lines_w);
  al_draw_line(box_X2, box_Y2+round(box_vis_H2*2/3), box_X2+box_vis_W2, box_Y2+round(box_vis_H2*2/3), al_map_rgb(0,0,0), lines_w);
  al_draw_text(font, al_map_rgb(0,0,0), box_X2+round(box_vis_W2*1/6), box_Y2+box_vis_H2+text_bias, ALLEGRO_ALIGN_CENTRE, "-T6");
  al_draw_text(font, al_map_rgb(0,0,0), box_X2+round(box_vis_W2*2/6), box_Y2+box_vis_H2+text_bias, ALLEGRO_ALIGN_CENTRE, "-T1");
  al_draw_text(font, al_map_rgb(0,0,0), box_X2+round(box_vis_W2*3/6), box_Y2+box_vis_H2+text_bias, ALLEGRO_ALIGN_CENTRE, "0");
  al_draw_text(font, al_map_rgb(0,0,0), box_X2+round(box_vis_W2*4/6), box_Y2+box_vis_H2+text_bias, ALLEGRO_ALIGN_CENTRE, "+T1");
  al_draw_text(font, al_map_rgb(0,0,0), box_X2+round(box_vis_W2*5/6), box_Y2+box_vis_H2+text_bias, ALLEGRO_ALIGN_CENTRE, "+T6");
  al_draw_text(font, al_map_rgb(0,0,0),  box_X2-text_bias, box_Y2+round(box_vis_H2*2/3)-FONTHEIGHT, ALLEGRO_ALIGN_RIGHT, "-W50");
  al_draw_text(font, al_map_rgb(0,0,0),  box_X2-text_bias, box_Y2+round(box_vis_H2*1/3)-FONTHEIGHT, ALLEGRO_ALIGN_RIGHT, "+W50");

  // vertical lines
  al_draw_line(box_X2+round(box_vis_W2*1/6), box_Y2, box_X2+round(box_vis_W2*1/6), box_Y2+box_vis_H2, al_map_rgb(0,0,0), lines_w);
  al_draw_line(box_X2+round(box_vis_W2*2/6), box_Y2, box_X2+round(box_vis_W2*2/6), box_Y2+box_vis_H2, al_map_rgb(0,0,0), lines_w);
  al_draw_line(box_X2+round(box_vis_W2*3/6), box_Y2, box_X2+round(box_vis_W2*3/6), box_Y2+box_vis_H2, al_map_rgb(0,0,0), lines_w);
  al_draw_line(box_X2+round(box_vis_W2*4/6), box_Y2, box_X2+round(box_vis_W2*4/6), box_Y2+box_vis_H2, al_map_rgb(0,0,0), lines_w);
  al_draw_line(box_X2+round(box_vis_W2*5/6), box_Y2, box_X2+round(box_vis_W2*5/6), box_Y2+box_vis_H2, al_map_rgb(0,0,0), lines_w);
}

#define g_drk 100 // dark gray
#define g_mid 130 // medium gray
#define g_lht 255 // light gray


bool manage_button(BUTTON* b, float x, float y, bool c, const ALLEGRO_FONT *font)
{
  bool released = false;
  // button state computation
  if(((int)x>(*b).x0)&((int)x<((*b).x0+(*b).w))&((int)y>(*b).y0)&((int)y<((*b).y0+(*b).h))) // if I'm over the BUTTON
  {
    if(c)
    {
      (*b).state = +1; // button is pressed
      (*b).clr = g_mid;
    }
    else
      (*b).state = -1; // button is NOT pressed (but I'm over it)
  }
  else
    (*b).state = 0;  // I'm not over the BUTTON

  if(!c)
    (*b).clr = g_drk;

  if(((*b).state == -1)&((*b).prev_state == +1))
  {
    released = true;
    (*b).clr = g_lht;
  }

  // button drawing
  al_draw_filled_rectangle((*b).x0, (*b).y0, (*b).x0+(*b).w, (*b).y0+(*b).h, al_map_rgb((*b).clr,(*b).clr,(*b).clr));
  al_draw_text(font, al_map_rgb(0,0,0), (*b).x0+(*b).w/2, (*b).y0+(*b).h/2-FONTHEIGHT/2, ALLEGRO_ALIGN_CENTRE, (*b).text);

  if(DEBUG)
    al_draw_filled_circle((*b).x0, (*b).y0, 3, al_map_rgb(255,0,0));  // For positioning

  (*b).prev_state = (*b).state;
  return released;
}

float manage_slider(SLIDER* s, float x, float y, bool c, const ALLEGRO_FONT *font)
{
  int newVal;
  bool unlock = false;
  // slider state computation
  if(((int)x>(*s).b.x0)&((int)x<((*s).b.x0+(*s).b.w))&((int)y>(*s).b.y0)&((int)y<((*s).b.y0+(*s).b.h))) // if I'm over the BUTTON
  {
    if(c)
      (*s).b.state = +1; // button is pressed
    else
      (*s).b.state = -1; // button is NOT pressed (but I'm over it)
  }
  else
    (*s).b.state = 0;  // I'm not over the BUTTON
  if(!c)
  {
    (*s).movable = false;
    (*s).b.clr = g_drk;
  }

  if(((*s).b.state == +1)&((*s).b.prev_state == -1))  // if I've clicked on the button I make it movable
  {
    (*s).movable = true;
    (*s).b.clr = g_mid;
  }


  if((*s).movable)
    if((*s).vertical)
    {
      newVal = y-(*s).b.h/2;
      if (newVal > (*s).y0)
        newVal = (*s).y0;
      else if(newVal < (*s).y0-(*s).l)
        newVal = (*s).y0-(*s).l;
      (*s).b.y0 = newVal;
      (*s).value = -((float)(newVal-(*s).y0)/((float)(*s).l))*((*s).max-(*s).min)+(*s).min;
      (*s).b.x0 = (*s).x0;
    }
    else
    {
      newVal = x-(*s).b.w/2;
      if (newVal < (*s).x0)
        newVal = (*s).x0;
      else if(newVal > (*s).x0+(*s).l)
        newVal = (*s).x0+(*s).l;
      (*s).b.x0 = newVal;
      (*s).value = ((float)(newVal-(*s).x0)/((float)(*s).l))*((*s).max-(*s).min)+(*s).min;
      (*s).b.y0 = (*s).y0;
    }
    else
      if((*s).vertical)
         (*s).b.y0 = (*s).y0-((float)((*s).value-(*s).min))/((float)((*s).max-(*s).min))*((float)(*s).l);
       else
       {
         (*s).b.x0 = (*s).x0+((float)((*s).value-(*s).min))/((float)((*s).max-(*s).min))*((float)(*s).l);
       }


  if((*s).vertical)
  {
    al_draw_line((*s).x0+(*s).b.w/2, (*s).y0+(*s).b.h+3, (*s).x0+(*s).b.w/2, (*s).y0-(*s).l-3, al_map_rgb(0,0,0), 3);
    al_draw_textf(font, al_map_rgb(0,0,0), (*s).x0+(*s).b.w/2, (*s).y0+(*s).b.h+6, ALLEGRO_ALIGN_CENTRE, "%i", (int)(*s).min);
    al_draw_textf(font, al_map_rgb(0,0,0), (*s).x0+(*s).b.w/2, (*s).y0-(*s).l-14, ALLEGRO_ALIGN_CENTRE, "%i", (int)(*s).max);
    al_draw_text(font, al_map_rgb(0,0,0), (*s).x0+(*s).b.w/2, (*s).y0-(*s).l-30, ALLEGRO_ALIGN_CENTRE, (*s).text);

  }
  else
  {
    al_draw_line((*s).x0-3, (*s).y0+(*s).b.h/2, (*s).x0+(*s).l+(*s).b.w+3, (*s).y0+(*s).b.h/2, al_map_rgb(0,0,0), 3);
    al_draw_textf(font, al_map_rgb(0,0,0), (*s).x0-6, (*s).y0+(*s).b.h/2-FONTHEIGHT/2, ALLEGRO_ALIGN_RIGHT, "%i", (int)(*s).min);
    al_draw_textf(font, al_map_rgb(0,0,0), (*s).x0+(*s).l+(*s).b.w+6, (*s).y0+(*s).b.h/2-FONTHEIGHT/2, ALLEGRO_ALIGN_LEFT, "%i", (int)(*s).max);
    al_draw_text(font, al_map_rgb(0,0,0), (*s).x0+((*s).l+(*s).b.w)/2, (*s).y0-10, ALLEGRO_ALIGN_CENTRE, (*s).text);

  }
  al_draw_filled_rectangle((*s).b.x0, (*s).b.y0, (*s).b.x0+(*s).b.w, (*s).b.y0+(*s).b.h, al_map_rgb((*s).b.clr,(*s).b.clr,(*s).b.clr));
  al_draw_textf(font, al_map_rgb(0,0,0), (*s).b.x0+(*s).b.w/2, (*s).b.y0+(*s).b.h/2-FONTHEIGHT/2, ALLEGRO_ALIGN_CENTRE, "%i", (int)(*s).value);

  if(DEBUG)
    al_draw_filled_circle((*s).x0, (*s).y0, 3, al_map_rgb(255,0,0));  // For positioning

  (*s).b.prev_state = (*s).b.state;
  return (*s).value;
}

void slider_constructor(SLIDER*s, float min, float max, int x0, int y0, int l, bool vertical, float value0, int bw, int bh, char* text)
{
  int bx, by;
  if(vertical)
  {
    bx = x0;
    by = y0-l*(value0-min)/(max-min);
  }
  else
  {
    bx = x0+l*(value0-min)/(max-min);
    by = y0;
  }
  BUTTON b = {bx, by, bw, bh, 0, 0, "\0", 0}; //  x0, y0, w, h, state, prev_state, text, color
  s->b = b;
  s->min = min;
  s->max = max;
  s->x0 = x0;
  s->y0 = y0;
  s->l = l;
  s->vertical = vertical;
  s->value = value0;
  s->movable = false;
  memcpy (s->text, text, 64);
}

void graph_constructor(GRAPH*gr, float ymin, float ymax, int x0, int y0, int w, int h, int buff_size, char* text_x_axis, char* text_y_axis, char* text_title)
{
  gr-> x0 = x0;
  gr-> y0 = y0;
  gr-> w = w;
  gr-> h = h;
  for(int i=0; i<MAX_GRAPH_WIDTH; i++)
    gr-> pts[i] = 0;
  gr-> pts_ind = 0;
  gr-> inc_buff = 0;
  gr-> pts_in_buff = 0;
  gr-> buff_size = buff_size;
  memcpy (gr->text_x_axis, text_x_axis, 64);
  memcpy (gr->text_y_axis, text_y_axis, 64);
  memcpy (gr->text_title, text_title, 64);
  gr-> fsyH = ymax;
  gr-> fsyL = ymin;
}



void button_constructor(BUTTON* b, int x0, int y0, int w, int h, char* text)
{
  b->x0 = x0;
  b->y0 = y0;
  b->w = w;
  b->h = h;
  b->state = 0;
  b->prev_state = 0;
  memcpy (b->text, text, 64);
  b->clr = 0;
}


float compute_pid(PID* p, STATE* s, float err)
{
  float force;
  p->integral += err;
  force = (p->Kp)*err + (p->Kd)*(err-(p->prev_err)) + (p->Ki)*(p->integral);
  p->prev_err = err;
  return force;
}

void plot_graph(GRAPH* gr, float v, const ALLEGRO_FONT *font, float dt, bool update)
{
  static int i, zero_point;
  static int fsxH, fsxL;

  if(update)
  {
    gr->inc_buff += v;
    gr->pts_in_buff++;
    if(gr->pts_in_buff >= gr->buff_size)
    {
      gr->pts_ind++;
      // gr->pts_ind %= gr->w;
      // gr->pts[gr->pts_ind] = round(((int)(((gr->inc_buff))/((float)(gr->buff_size))))/(gr->fsyH)*gr->h);
      gr->pts[gr->pts_ind%gr->w] = round((int)(((((gr->inc_buff))/((float)(gr->buff_size)))-(float)(gr->fsyL))/(float)(gr->fsyH-gr->fsyL)*(float)(gr->h)));
      if(gr->pts[gr->pts_ind%gr->w]>gr->h)
        gr->pts[gr->pts_ind%gr->w] = gr->h-1;
      else if(gr->pts[gr->pts_ind%gr->w]<0)
        gr->pts[gr->pts_ind%gr->w] = 0;
      gr->pts_in_buff = 0;
      gr->inc_buff = 0;
    }
  }


  al_draw_filled_rectangle(gr->x0, gr->y0, gr->x0+gr->w, gr->y0-gr->h, al_map_rgb(100,100,100));  // background
  if((gr->fsyH > 0)&&(gr->fsyL < 0))
  {
    zero_point = gr->y0-((-gr->fsyL)/(gr->fsyH-gr->fsyL)*gr->h);
    al_draw_line(gr->x0, zero_point, gr->x0+gr->w, zero_point, al_map_rgb(80,80,80), 1);  // 0 line
    al_draw_text(font, al_map_rgb(0, 0, 0), gr->x0-2, zero_point-FONTHEIGHT/2, ALLEGRO_ALIGN_RIGHT, "0");

  }
  if(gr->pts_ind <= gr->w)
  {
    fsxH = round(dt*gr->buff_size*gr->w);
    fsxL = 0;
    for(i=1; i<gr->pts_ind; i++)
    {
      //   al_draw_pixel(gr->x0+i, gr->y0-(gr->pts[i]), al_map_rgb(255,0,0)); // pac-man graph pts
      al_draw_line(gr->x0+i, gr->y0-(gr->pts[i]), gr->x0+i-1, gr->y0-(gr->pts[i-1]), al_map_rgb(255,100,0), 2);  // pac-man graph lines
    }
  }
  else
  {
    fsxH = round(dt*gr->buff_size*gr->pts_ind);
    fsxL = round(dt*gr->buff_size*(gr->pts_ind-gr->w));
    for(i=2; i<gr->w; i++)
    {
      // al_draw_pixel(gr->x0+i, gr->y0-(gr->pts[(gr->pts_ind+i)%gr->w]), al_map_rgb(255,0,0));  // scrolling graph pts
      al_draw_line(gr->x0+i, gr->y0-(gr->pts[(gr->pts_ind+i)%gr->w]), gr->x0+(i+gr->w-1)%gr->w, gr->y0-(gr->pts[(gr->pts_ind+(i+gr->w-1))%gr->w]), al_map_rgb(255,100,0), 2);  // scrolling graph lines
    }
  }

  al_draw_rectangle(gr->x0, gr->y0, gr->x0+gr->w, gr->y0-gr->h, al_map_rgb(0,0,0), 2); // cornice
  al_draw_textf(font, al_map_rgb(0, 0, 0), gr->x0+gr->w, gr->y0+2, ALLEGRO_ALIGN_CENTRE, "%i", fsxH);
  al_draw_textf(font, al_map_rgb(0, 0, 0), gr->x0, gr->y0+2, ALLEGRO_ALIGN_CENTRE, "%i", fsxL);
  al_draw_text(font, al_map_rgb(0, 0, 0), gr->x0+gr->w/2, gr->y0+2, ALLEGRO_ALIGN_CENTRE, gr->text_x_axis);

  al_draw_textf(font, al_map_rgb(0, 0, 0), gr->x0-2, gr->y0-gr->h, ALLEGRO_ALIGN_RIGHT, "%i", (int)round(gr->fsyH));
  al_draw_textf(font, al_map_rgb(0, 0, 0), gr->x0-2, gr->y0-FONTHEIGHT, ALLEGRO_ALIGN_RIGHT, "%i", (int)round(gr->fsyL));
  al_draw_text(font, al_map_rgb(0, 0, 0), gr->x0-2, gr->y0-gr->h/2-FONTHEIGHT/2, ALLEGRO_ALIGN_RIGHT, gr->text_y_axis);

  al_draw_text(font, al_map_rgb(0, 0, 0), gr->x0+gr->w/2, gr->y0-gr->h-2-FONTHEIGHT, ALLEGRO_ALIGN_CENTRE, gr->text_title);

  if(DEBUG)
    al_draw_filled_circle(gr->x0, gr->y0, 3, al_map_rgb(255,0,0));  // For positioning
}

#define charge_coeff  5000//15
float user_force(float x, float y, bool click, float noise_coeff, STATE state, DIM dim, const ALLEGRO_FONT *font)
{
  float force = 0;
  static int x0, y0, charge;
  static bool prev_click = false;
  static bool block = false;

  if(click && !prev_click)
  {
    if(y < (cart_Y_pose+(wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset))
    {
      prev_click = true;
      if(!block)
      {
        x0 = x;
        y0 = y;
        charge = 0;
      }
    }
    else
    {
      block = true;
      charge = 0;
    }
  }
  if(click && prev_click && !block)
  {
    charge = sqrt(pow(x-x0,2)+pow(y-y0,2));
    al_draw_filled_circle(x0, y0, charge, al_map_rgb(255,100,0));
  }
  if(!click && prev_click)
  {
    if(!block)
    {
      force = 1/((float)(x0-(state.x+1)*dW/2));
      // force = force>1 ? 1 : force<-1 ? -1 : force;
      force = -(force*charge*charge_coeff);
      // force = sgn(force) * ((1-sgn(force)*(force))*charge*charge_coeff);
      // force = -sgn(force) * pow(((1-sgn(force)*(force))*(float)charge*charge_coeff),2);
      prev_click = false;
      al_draw_filled_circle(x0, y0, charge, al_map_rgb(100,150,255));
    }
    else
    {
      block = false;
    }
  }

  if(!click && !prev_click && (y < (cart_Y_pose+(wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset)))
  {
    block = false;
  }

  click = false;

  force += randf(-1,1)*noise_coeff;
  al_draw_text(font, al_map_rgb(255,100,0), 10, cart_Y_pose+(wheel_diameter*dim.Mc*Mc_coeff)/2+wheel_offset+FONTHEIGHT, ALLEGRO_ALIGN_LEFT, "Click and drag above this line to produce an explosion.");

  return force;
}

bool save_to_file(char* file_name, float* Q, float* epsilon, float* epsilon_decay, float* gamma_, float* alpha, int* action_multiplier, float* noise_coeff, DIM* dim, PID* pid_theta, PID* pid_x, GRAPH* g_totRew, GRAPH* g_instRew, GRAPH* g_frame, GRAPH* g_tdErr)
{
  FILE * fp;
  int i;

  fp = fopen (file_name, "w");   // open the file for writing
  if (fp == NULL)
    return true;

  // Store Q
  for(i=0; i<(NBX*NBV*NBT*NBW*2); i++)
    fprintf(fp, "%f, ", Q[i]);
  fprintf(fp, " <-- Q\n");
  // Store epsilon, epsilon_decay, gamma_, alpha, action_multiplier, noise_coeff
  fprintf(fp, "%f, <-- epsilon\n%f, <-- epsilon_decay\n%f, <-- gamma_\n%f, <-- alpha\n%i, <-- action_multiplier\n%f, <-- noise_coeff\n", *epsilon, *epsilon_decay, *gamma_, *alpha, *action_multiplier, *noise_coeff);
  // Store dim
  fprintf(fp, "%f, <-- dim.Mc\n%f, <-- dim.Mp\n%f, <-- dim.Lp\n", dim->Mc, dim->Mp, dim->Lp);
  // Store pid theta
  fprintf(fp, "%f, <-- pid_theta.Kp\n%f, <-- pid_theta.Ki\n%f, <-- pid_theta.Kd\n", pid_theta->Kp, pid_theta->Ki, pid_theta->Kd);
  // Store pid x
  fprintf(fp, "%f, <-- pid_x.Kp\n%f, <-- pid_x.Ki\n%f, <-- pid_x.Kd\n", pid_x->Kp, pid_x->Ki, pid_x->Kd);

  // Store g_totRew.pts
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
    fprintf(fp, "%i, ", g_totRew->pts[i]);
  fprintf(fp, " <-- g_totRew.pts\n");
  // Store g_totRew.pts_ind
  fprintf(fp, "%li, <-- g_totRew.pts_ind\n", g_totRew->pts_ind);

  // Store g_instRew.pts
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
    fprintf(fp, "%i, ", g_instRew->pts[i]);
  fprintf(fp, " <-- g_instRew.pts\n");
  // Store g_instRew.pts_ind
  fprintf(fp, "%li, <-- g_instRew.pts_ind\n", g_instRew->pts_ind);

  // Store g_frame.pts
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
    fprintf(fp, "%i, ", g_frame->pts[i]);
  fprintf(fp, " <-- g_frame.pts\n");
  // Store g_frame.pts_ind
  fprintf(fp, "%li, <-- g_frame.pts_ind\n", g_frame->pts_ind);

  // Store g_tdErr.pts
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
    fprintf(fp, "%i, ", g_tdErr->pts[i]);
  fprintf(fp, " <-- g_tdErr.pts\n");
  // Store g_tdErr.pts_ind
  fprintf(fp, "%li, <-- g_tdErr.pts_ind\n", g_tdErr->pts_ind);

  fclose (fp);  // close the file

  return false;
}

const char* getfield(char* line, int num)
{
    const char* tok;
    for (tok = strtok(line, ","); tok && *tok; tok = strtok(NULL, ","))
      if (!--num)
        return tok;
    return NULL;
}

#define MAX_FILE_LENGHT 16384
bool load_from_file(char* file_name, float* Q, float* epsilon, float* epsilon_decay, float* gamma_, float* alpha, int* action_multiplier, float* noise_coeff, DIM* dim, PID* pid_theta, PID* pid_x, GRAPH* g_totRew, GRAPH* g_instRew, GRAPH* g_frame, GRAPH* g_tdErr)
{
  FILE *fp;
  char line[MAX_FILE_LENGHT];
  char* tmp;
  int i;

  fp = fopen(file_name, "r");

  if (fp == NULL)
  {
    printf("Could not open file %s",file_name);
    return true;  // if file is not present, return error.
  }

  // Load Q
  fgets(line, MAX_FILE_LENGHT, fp);
  for(i=0; i<(NBX*NBV*NBT*NBW*2); i++)
  {
    tmp = strdup(line);
    Q[i] = atof(getfield(tmp, i+1));
    free(tmp);

  }
  // Load epsilon
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  *epsilon = atof(getfield(tmp, 1));
  free(tmp);
  // Load epsilon_decay
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  *epsilon_decay = atof(getfield(tmp, 1));
  free(tmp);
  // Load gamma_
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  *gamma_ = atof(getfield(tmp, 1));
  free(tmp);
  // Load alpha
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  *alpha = atof(getfield(tmp, 1));
  free(tmp);
  // Load action_multiplier
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  *action_multiplier = atof(getfield(tmp, 1));
  free(tmp);
  // Load noise_coeff
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  *noise_coeff = atof(getfield(tmp, 1));
  free(tmp);
  // Load dim.Mc
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  dim->Mc = atof(getfield(tmp, 1));
  free(tmp);
  // Load dim.Mp
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  dim->Mp = atof(getfield(tmp, 1));
  free(tmp);
  // Load dim.Lp
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  dim->Lp = atof(getfield(tmp, 1));
  free(tmp);
  // Load pid_theta.Kp
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  pid_theta->Kp = atof(getfield(tmp, 1));
  free(tmp);
  // Load pid_theta.Ki
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  pid_theta->Ki = atof(getfield(tmp, 1));
  free(tmp);
  // Load pid_theta.Kd
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  pid_theta->Kd = atof(getfield(tmp, 1));
  free(tmp);
  // Load pid_x.Kp
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  pid_x->Kp = atof(getfield(tmp, 1));
  free(tmp);
  // Load pid_x.Ki
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  pid_x->Ki = atof(getfield(tmp, 1));
  free(tmp);
  // Load pid_x.Kd
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  pid_x->Kd = atof(getfield(tmp, 1));
  free(tmp);

  // Load g_totRew.pts
  fgets(line, MAX_FILE_LENGHT, fp);
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
  {
    tmp = strdup(line);
    g_totRew->pts[i] = atof(getfield(tmp, i+1));
    free(tmp);
  }
  // Load g_totRew.pts_ind
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  g_totRew->pts_ind = atof(getfield(tmp, 1));
  free(tmp);

  // Load g_instRew.pts
  fgets(line, MAX_FILE_LENGHT, fp);
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
  {
    tmp = strdup(line);
    g_instRew->pts[i] = atof(getfield(tmp, i+1));
    free(tmp);
  }
  // Load g_instRew.pts_ind
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  g_instRew->pts_ind = atof(getfield(tmp, 1));
  free(tmp);

  // Load g_frame.pts
  fgets(line, MAX_FILE_LENGHT, fp);
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
  {
    tmp = strdup(line);
    g_frame->pts[i] = atof(getfield(tmp, i+1));
    free(tmp);
  }
  // Load g_frame.pts_ind
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  g_frame->pts_ind = atof(getfield(tmp, 1));
  free(tmp);

  // Load g_tdErr.pts
  fgets(line, MAX_FILE_LENGHT, fp);
  for(i=0; i<MAX_GRAPH_WIDTH; i++)
  {
    tmp = strdup(line);
    g_tdErr->pts[i] = atof(getfield(tmp, i+1));
    free(tmp);
  }
  // Load g_tdErr.pts_ind
  fgets(line, MAX_FILE_LENGHT, fp);
  tmp = strdup(line);
  g_tdErr->pts_ind = atof(getfield(tmp, 1));
  free(tmp);

  fclose(fp);
  return false;
}
*/