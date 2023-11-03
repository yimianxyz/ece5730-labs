/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 15 --> PUSHBUTTON
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
fix15 complementary_angle = int2fix15(0);
fix15 accel_angle = int2fix15(0);
fix15 gyro_angle_delta = int2fix15(0);
fix15 filt_ax =  int2fix15(0);
fix15 filt_ay =  int2fix15(0);

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

char str[40];

fix15 Kp = int2fix15(150);
fix15 Ki = float2fix15(0.2);
fix15 Kd = int2fix15(20000);
fix15 desired_angle = int2fix15(30);
fix15 error_ang = 0;
fix15 last_error = 0;
fix15 integral_cntl = 0;
fix15 I_MAX = int2fix15(2500);

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0
uint slice_num ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;
volatile int motor_disp ;
const int CONTROL_MAX = 5000;
const int CONTROL_MIN = 0;

uint32_t start_time = 0;


void gpio_callback(uint gpio, uint32_t events) {
    if(events & GPIO_IRQ_EDGE_FALL){
        // printf("FALLING EDGE\n");
        start_time = 0;
        desired_angle = int2fix15(0);
    } else if(events & GPIO_IRQ_EDGE_RISE){
        // printf("RISING EDGE\n");
        desired_angle = int2fix15(90);
        start_time = time_us_32();
        //sleep for 5 second
        // sleep_ms(5000);
        // desired_angle = int2fix15(120);
        // sleep_ms(5000);
        // desired_angle = int2fix15(60);
        // sleep_ms(5000);
        // desired_angle = int2fix15(90);


    }
}



// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    if(start_time != 0){
        if(time_us_32() - start_time > 5000000 && time_us_32() - start_time < 10000000){
            desired_angle = int2fix15(120);
        } else if(time_us_32() - start_time > 10000000 && time_us_32() - start_time < 15000000){
            desired_angle = int2fix15(60);
        } else if(time_us_32() - start_time > 15000000 && time_us_32() - start_time < 20000000){
            desired_angle = int2fix15(90);
        }
    }

    // accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), oneeightyoverpi);
    filt_ax = filt_ax + (acceleration[1] - filt_ax) >> 4;
    filt_ay = filt_ay + (acceleration[2] - filt_ay) >> 4;
    accel_angle = multfix15(float2fix15(atan2(-filt_ax, filt_ay)) + float2fix15(M_PI), oneeightyoverpi);

    gyro_angle_delta = multfix15(gyro[0], zeropt001);

    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    last_error = error_ang;

    error_ang = desired_angle - complementary_angle + int2fix15(80); //oldMin

    integral_cntl = integral_cntl + error_ang;

    // if( (error_ang < int2fix15(0) && last_error >= int2fix15(0) ) || (error_ang >= int2fix15(0) && last_error < int2fix15(0) ) ){
    //     if(integral_cntl > 20000){
    //         integral_cntl = 0;
    //     }
    // }

    if(integral_cntl > I_MAX){
        integral_cntl = I_MAX;
    }
    if(integral_cntl < -I_MAX){
        integral_cntl = -I_MAX;
    }

    control = fix2int15(multfix15(Kp,error_ang)) + fix2int15(multfix15(Kd, error_ang - last_error)) + fix2int15(multfix15(Ki, integral_cntl));

    if(control > CONTROL_MAX){
        control = CONTROL_MAX;
    } else if(control < CONTROL_MIN){
        control = CONTROL_MIN;
    }

    // Update duty cycle
    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
    }

    motor_disp += (control - motor_disp) >> 6;

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 200. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = 80. ;
    static float OldMax = 280. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "90") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "180") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "MAX") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "MIN") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            //write string on VGA
            setTextColor2(WHITE, BLACK) ;
            sprintf(str, "%f", fix2float15(complementary_angle)-OldMin);
            // sprintf(str, "%f", fix2float15(integral_cntl));
            setCursor(65, 0) ;
            setTextSize(1) ;
            writeString("complementary angle:") ;
            writeString(str) ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[0])*120.0)-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[1])*120.0)-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[2])*120.0)-OldMin)/OldRange)), GREEN) ;

            drawPixel(xcoord, 430 - (int)(0.75*((float)((fix2float15(complementary_angle))-OldMin))), RED) ;

            //printf("%f\n", fix2float15(complementary_angle));

           

            // Draw top plot
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[1]))-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)(motor_disp-0)/5000)), GREEN) ;



            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static float user_input ;
    static float f_user_input ;
    static int uiSelected = 0;
      while(1) {


        printf("\n\nDesired Angle: %f\n", fix2float15(desired_angle));
        printf("Kp: %f\n", fix2float15(Kp));
        printf("Ki: %f\n", fix2float15(Ki));
        printf("Kd: %f\n", fix2float15(Kd));
        if(uiSelected == 0){
            printf("desired_angle = ");
        }
        else if(uiSelected == 1){
            printf("Kp = ");
        }
        else if(uiSelected == 2){
            printf("Ki = ");
        }
        else if(uiSelected == 3){
            printf("Kd = ");
        }

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
            desired_angle = float2fix15(user_input);
          }
          else if(uiSelected == 1){
            Kp = float2fix15(user_input);
          }
          else if(uiSelected == 2){
            Ki = float2fix15(user_input);
          }
          else if(uiSelected == 3){
            Kd = float2fix15(user_input);
          }

        } else {
          uiSelected = (uiSelected + 1) % 4;
        }
      } // END WHILE(1)

    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    gpio_init(15);
    gpio_set_dir(15, GPIO_IN);
    gpio_pull_up(15);
    gpio_set_irq_enabled_with_callback(15, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
