
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define float2fix(a) ((fix15)((a) * 32768.0))
#define fix2float(a) ((float)(a) / 32768.0)
#define sqrtfix(a) (float2fix(sqrt(fix2float(a))))

// uS per frame
#define FRAME_RATE 33000

#define MAX_SPEED 6
#define MIN_SPEED 3

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

#define NUM_OF_BOIDS 500
#define NUM_OF_BOIDS_ON_CORE0 240


// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// Screen edges detection
#define hitBottomEdge(b) (b>int2fix15(SCREEN_HEIGHT))
#define hitTopEdge(b) (b<int2fix15(0))
#define hitLeftEdge(a) (a<int2fix15(0))
#define hitRightEdge(a) (a>int2fix15(SCREEN_WIDTH))


// the color of the boid
char color = WHITE ;

int uiSelected = 0;
int uiRefreshDone = 0;

// modes
fix15 wallMode = int2fix15(0);
fix15 predatorMode = int2fix15(0);

//factors
fix15 turnfactor = float2fix15(0.2);
fix15 visualrange = float2fix15(40);
fix15 protectedrange = float2fix15(8);
fix15 centeringfactor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matcingfactor = float2fix15(0.05);
fix15 predatorturnfactor = float2fix15(0.5);
fix15 predatorRange = float2fix15(100);


// alpha max beta min parameters
fix15 alpha = float2fix15(0.96);
fix15 beta = float2fix15(0.398);

// min and max functions

static inline fix15 max(fix15 a, fix15 b) {
  return (a > b ) ? a : b;
}

static inline fix15 min(fix15 a, fix15 b) {
  return (a > b ) ? b : a;
}


// Boid struct
typedef struct boid{
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
}boid;

// Array of boids
boid boids[NUM_OF_BOIDS];
boid predator;

char str[40];

int g_core1_spare_time = 0;

// Init boids
void initBoids(){
  for(int i = 0; i < NUM_OF_BOIDS; i++){
    boids[i].x = int2fix15(rand() % (540 - 100) + 100);
    boids[i].y = int2fix15(rand() % (380 - 100) + 100);
    boids[i].vx = int2fix15((rand() % (MAX_SPEED - MIN_SPEED) + MIN_SPEED) - (MAX_SPEED + MIN_SPEED)*(rand()%2));
    boids[i].vy = int2fix15((rand() % (MAX_SPEED - MIN_SPEED) + MIN_SPEED) - (MAX_SPEED + MIN_SPEED)*(rand()%2));
  }
  predator.x = int2fix15(rand() % (540 - 100) + 100);
  predator.y = int2fix15(rand() % (380 - 100) + 100);
  predator.vx = int2fix15((rand() % (MAX_SPEED - MIN_SPEED) + MIN_SPEED) - (MAX_SPEED + MIN_SPEED)*(rand()%2));
  predator.vy = int2fix15((rand() % (MAX_SPEED - MIN_SPEED) + MIN_SPEED) - (MAX_SPEED + MIN_SPEED)*(rand()%2));
}





// Draw the boundaries
void drawArena(char color) {
  drawVLine(100, 100, 280, color) ;
  drawVLine(540, 100, 280, color) ;
  drawHLine(100, 100, 440, color) ;
  drawHLine(100, 380, 440, color) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  if(wallMode == int2fix15(0)){
    // Reverse direction if we've hit a wall
    if (hitTop(*y)) {
      *vy = (*vy + turnfactor) ;
    }
    if (hitBottom(*y)) {
      *vy = (*vy - turnfactor) ;
    } 
    if (hitRight(*x)) {
      *vx = (*vx - turnfactor) ;
    }
    if (hitLeft(*x)) {
      *vx = (*vx + turnfactor) ;
    } 
  }else{
    // Move to opposite side of screen if we've reach the screen edge
    if (hitTopEdge(*y)) {
      *y = int2fix15(SCREEN_HEIGHT) ;
    }
    if (hitBottomEdge(*y)) {
      *y = int2fix15(0) ;
    }
    if (hitRightEdge(*x)) {
      *x = int2fix15(0) ;
    }
    if (hitLeftEdge(*x)) {
      *x = int2fix15(SCREEN_WIDTH) ;
    }
  }
}


//limit speed
void limit_speed(fix15* vx, fix15* vy){
  //get rid of sqrt function, use Alpha max plus beta min instead.
  //alpha : 1
  //beta  : 1/4 (right shift 2)
  //fix15 speed = sqrtfix(multfix15(*vx, *vx) + multfix15(*vy, *vy));
  fix15 speed = multfix15(max(absfix15(*vx), absfix15(*vy)), alpha)  + multfix15(min(absfix15(*vx), absfix15(*vy)), beta); 
  if(speed > int2fix15(MAX_SPEED)){
    *vx = divfix(multfix15(*vx, int2fix15(MAX_SPEED)), speed);
    *vy = divfix(multfix15(*vy, int2fix15(MAX_SPEED)), speed);
  }
  else if(speed < int2fix15(MIN_SPEED)){
    *vx = divfix(multfix15(*vx, int2fix15(MIN_SPEED)), speed);
    *vy = divfix(multfix15(*vy, int2fix15(MIN_SPEED)), speed);
  }
}

// Update boid
void update_boid(int i){

  fix15 *x = &boids[i].x;
  fix15 *y = &boids[i].y;
  fix15 *vx = &boids[i].vx;
  fix15 *vy = &boids[i].vy;

  fix15 xpos_avg = int2fix15(0);
  fix15 ypos_avg = int2fix15(0);
  fix15 xvel_avg = int2fix15(0);
  fix15 yvel_avg = int2fix15(0);
  fix15 close_dx = int2fix15(0);
  fix15 close_dy = int2fix15(0);
  fix15 neighboring_boids = int2fix15(0);

  for(int j = 0; j < NUM_OF_BOIDS; j++){
    if(i != j){
      fix15 dx = *x - boids[j].x;
      // if(dx > int2fix15(320)) dx -= int2fix15(640);
      // if(dx < int2fix15(-320)) dx += int2fix15(640);
      fix15 dy = *y - boids[j].y;
      // if(dy > int2fix15(240)) dy -= int2fix15(480);
      // if(dy < int2fix15(-240)) dy += int2fix15(480);
      
      
      if(dx < visualrange && dy < visualrange && dx > -visualrange && dy > -visualrange){
        fix15 square_dist = (multfix15(dx, dx) + multfix15(dy, dy));

        if(square_dist < multfix15(protectedrange,protectedrange)){
          close_dx += dx;
          close_dy += dy;
        }
        else if (square_dist < multfix15(visualrange,visualrange)){
          xpos_avg += boids[j].x;
          ypos_avg += boids[j].y;
          xvel_avg += boids[j].vx;
          yvel_avg += boids[j].vy;
          neighboring_boids += int2fix15(1);
        }
      }
      
    }
  }
  // printf("%d",neighboring_boids);
  if (neighboring_boids > 0){
    xpos_avg = divfix(xpos_avg, neighboring_boids);
    ypos_avg = divfix(ypos_avg, neighboring_boids);
    xvel_avg = divfix(xvel_avg, neighboring_boids);
    yvel_avg = divfix(yvel_avg, neighboring_boids);

    // fix15 dx = xpos_avg - *x;
    // fix15 dy = ypos_avg - *y;
    // fix15 dvx = xvel_avg - *vx;
    // fix15 dvy = yvel_avg - *vy;

    *vx += multfix15(xpos_avg - *x, centeringfactor) + multfix15(xvel_avg - *vx, matcingfactor);
    *vy += multfix15(ypos_avg - *y, centeringfactor) + multfix15(yvel_avg - *vy, matcingfactor);
  }

  *vx += multfix15(close_dx, avoidfactor);
  *vy += multfix15(close_dy, avoidfactor);

  if(predatorMode){
    fix15 dx = *x - predator.x;
    fix15 dy = *y - predator.y;

    if(dx < predatorRange && dy < predatorRange && dx > -predatorRange && dy > -predatorRange){
      fix15 square_dist = (multfix15(dx, dx) + multfix15(dy, dy));

      if(square_dist < multfix15(predatorRange,predatorRange)){
        if(dy > int2fix15(0)){
          *vy += predatorturnfactor;
        }
        if(dy < int2fix15(0)){
          *vy -= predatorturnfactor;
        }
        if(dx > int2fix15(0)){
          *vx += predatorturnfactor;
        }
        if(dx < int2fix15(0)){
          *vx -= predatorturnfactor;
        }
      }
    }
    

  }


  wallsAndEdges(x, y, vx, vy);

  limit_speed(vx, vy);

  *x += *vx;
  *y += *vy;

  // pos double check to make sure we're not out of bounds
  // if(*x > int2fix15(SCREEN_WIDTH)){
  //   *x = int2fix15(SCREEN_WIDTH);
  // }
  // else if(*x < int2fix15(0)){
  //   *x = int2fix15(0);
  // }
  // if(*y > int2fix15(SCREEN_HEIGHT)){
  //   *y = int2fix15(SCREEN_HEIGHT);
  // }
  // else if(*y < int2fix15(0)){
  //   *y = int2fix15(0);
  // }
}

void update_predator(){
  fix15 *x = &predator.x;
  fix15 *y = &predator.y;
  fix15 *vx = &predator.vx;
  fix15 *vy = &predator.vy;

  wallsAndEdges(x, y, vx, vy);
  limit_speed(vx, vy);

  *x += *vx;
  *y += *vy;

}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static float user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    // sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        static float f_user_input ;
        // print prompt
        // sprintf(pt_serial_out_buffer, "\n\ninput a new value = ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        f_user_input = user_input ;
        // convert input string to floating point number
        sscanf( pt_serial_in_buffer, "%f", &user_input);
        
        if(f_user_input != user_input){

          if(uiSelected == 0){
            wallMode = float2fix15(user_input);
          }
          else if(uiSelected == 1){
            visualrange = float2fix15(user_input);
          }
          else if(uiSelected == 2){
            protectedrange = float2fix15(user_input);
          }
          else if(uiSelected == 3){
            centeringfactor = float2fix15(user_input);
          }
          else if(uiSelected == 4){
            avoidfactor = float2fix15(user_input);
          }
          else if(uiSelected == 5){
            matcingfactor = float2fix15(user_input);
          }
          else if(uiSelected == 6){
            predatorMode = float2fix15(user_input);
          }
          else if(uiSelected == 7){
            predatorturnfactor = float2fix15(user_input);
          }
          else if(uiSelected == 8){
            predatorRange = float2fix15(user_input);
          }

        } else {
          uiSelected = (uiSelected + 1) % 9;
        }

        uiRefreshDone = 0;
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;
    
    // Init boids
    // initBoids();

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;    

      // erase the predator
      drawRect(fix2int15(predator.x), fix2int15(predator.y), 4, 4, BLACK);
      // update predator's position and velocity
      update_predator();
      // draw the predator at its new position
      drawRect(fix2int15(predator.x), fix2int15(predator.y), 4, 4, predatorMode?RED:BLACK);


      for (int i = 0; i < NUM_OF_BOIDS_ON_CORE0; i++){
        // erase boid
        drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, BLACK);
        // update boid's position and velocity
        update_boid(i);
        // draw the boid at its new position
        drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, color); 
      }
      // draw the boundaries
      drawArena(wallMode?BLACK:WHITE) ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;


      setTextColor2(WHITE, BLACK) ;
      sprintf(str, "%d",NUM_OF_BOIDS);
      setCursor(65, 0) ;
      setTextSize(1) ;
      writeString("Number of boids:") ;
      writeString(str) ;

      sprintf(str, "%d",FRAME_RATE);
      setCursor(65, 10) ;
      writeString("Frame rate:") ;
      writeString(str) ;

      setCursor(65, 20) ;
      writeString("Elapsed time:") ;
      sprintf(str, "%d",time_us_32()/1000000);
      writeString(str) ;

      setCursor(65, 30) ;
      writeString("Spare time 0:") ;
      sprintf(str, "%d",spare_time);
      writeString(str) ;

      setCursor(65, 40) ;
      writeString("Spare time 1:") ;
      sprintf(str, "%d",g_core1_spare_time);
      writeString(str) ;

      if(!uiRefreshDone){
        printf("\x1b[2J\x1b[1;1H");

        if(uiSelected == 0){
          printf("->");
        }
        printf("Wall Mode: %d\n", fix2int15(wallMode));

        if(uiSelected == 1){
          printf("->");
        }
        printf("Visual Range: %f\n", fix2float15(visualrange));

        if(uiSelected == 2){
          printf("->");
        }
        printf("Protected Range: %f\n", fix2float15(protectedrange));

        if(uiSelected == 3){
          printf("->");
        }
        printf("Centering Factor: %f\n", fix2float15(centeringfactor));

        if(uiSelected == 4){
          printf("->");
        }
        printf("Avoid Factor: %f\n", fix2float15(avoidfactor));

        if(uiSelected == 5){
          printf("->");
        }
        printf("Matching Factor: %f\n", fix2float15(matcingfactor));

        if(uiSelected == 6){
          printf("->");
        }
        printf("Predator Mode: %d\n", fix2int15(predatorMode));

        if(uiSelected == 7){
          printf("->");
        }
        printf("Predator Turn Factor: %f\n", fix2float15(predatorturnfactor));

        if(uiSelected == 8){
          printf("->");
        }
        printf("Predator Range: %f\n", fix2float15(predatorRange));



        printf("\n\nPlease input a new value = ");
        uiRefreshDone = 1;
      }
      
      //print
      // printf("Number of boids: %d\n" ,NUM_OF_BOIDS) ; 
      // printf("Elapsed time: %d\n" ,time_us_32()/1000000) ; 
      // printf("Frame rate: %d\n" ,FRAME_RATE) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;


    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;  


      for (int i = NUM_OF_BOIDS_ON_CORE0; i < NUM_OF_BOIDS; i++){
        // erase boid
        drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, BLACK);
        // update boid's position and velocity
        update_boid(i);
        // draw the boid at its new position
        drawRect(fix2int15(boids[i].x), fix2int15(boids[i].y), 2, 2, color); 
      }


      // // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // // yield for necessary amount of time

      g_core1_spare_time = spare_time;

      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  const uint32_t sys_clock = 250000;
  set_sys_clock_khz(sys_clock, true);
  stdio_init_all() ;
  

  // initialize VGA
  initVGA() ;

  // init boids
  initBoids();

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
