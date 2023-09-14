/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * Keypad Demo
 * 
 * KEYPAD CONNECTIONS
 *  - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
 *  - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
 *  - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
 *  - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
 *  - GPIO 13  -->     Pin 5 (button col 1)
 *  - GPIO 14  -->     Pin 6 (button col 2)
 *  - GPIO 15  -->     Pin 7 (button col 3)
 * 
 * VGA CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 * 
 * SERIAL CONNECTIONS
 *  - GPIO 0        -->     UART RX (white)
 *  - GPIO 1        -->     UART TX (green)
 *  - RP2040 GND    -->     UART GND
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"

// VGA graphics library
#include "vga_graphics.h"
#include "pt_cornell_rp2040_v1.h"

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 44000            // sample rate

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             1000
#define DECAY_TIME              1000
#define SUSTAIN_TIME            3720
#define BEEP_DURATION           5720
#define BEEP_REPEAT_INTERVAL    4000

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

#define MEASURE_ISR_GPIO      16

#define mask 0x000000000000000F


// Two variables to store core number
volatile int corenum_0  ;

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;

// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12


unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;


char keytext[40];
int prev_key = 0;

//play mode/ record mode
volatile enum {PLAY, RECORD} mode = PLAY;

//queue array for play and record
volatile int queue_play[100];
volatile int queue_play_length = 0;
volatile int queue_record[100];
volatile int queue_record_length = 0;

void copy_queue(int *queue1, int *queue2){
    int i;
    for(i = 0; i < queue1[0]; i++){
        queue2[i] = queue1[i];
    }
}



// State machine variables to control beeps
volatile enum {BEEP_OFF, BEEP_ON} beep_state = BEEP_OFF ;

//keypad type 1 or 2
volatile int type;

// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t) {
    
    if (beep_state == BEEP_OFF) return true ;

    //set gpio to 1
    gpio_put(MEASURE_ISR_GPIO, 1) ;

    if (STATE_0 == 0) {
        // Update phase_incr_main_0 to change frequency
        if(type == 1){
            phase_incr_main_0 = ((-260*sin((-3.14/5720)*count_0) + 1740)*two32)/Fs ;
        }
        else if(type == 2){
            phase_incr_main_0 = (((1.84*(0.0001)*count_0*count_0) + 2000)*two32)/Fs ;
        }
        else {
            phase_incr_main_0 = (400.0*two32)/Fs ;
        }

        // DDS phase and sine table lookup
        phase_accum_main_0 += phase_incr_main_0  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 1 ;
            count_0 = 0 ;

            type = queue_play[queue_play_length-1];
            queue_play_length--;


            printf("\nxx%x",type);

            if(queue_play_length <= 0){
                beep_state = BEEP_OFF ;
            }

        }
    }

    // State transition?
    else {
        current_amplitude_0 = 0 ;
        STATE_0 = 0 ;
        count_0 = 0 ;
        type = queue_play[queue_play_length-1];
        printf("\nyy%x",type);
    }

    // retrieve core number of execution
    corenum_0 = get_core_num() ;

    //set gpio to 0
    gpio_put(MEASURE_ISR_GPIO, 0) ;

    return true;
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static int fi ;
    static uint32_t keypad ;

    
    // State machine variables for debouncing
    static enum { NoPressed, MaybePressed, Pressed, MaybeNoPressed } state = NoPressed ;
    static int possible_key = -1 ;
    static int confirmed_key = -1 ;


    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ; 
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if (keypad & button) break ;
        }
        // If we found a button . . .
        if (keypad & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;


        // Debounce the button
        switch (state) {
            case NoPressed:
                if (i != -1) {
                    state = MaybePressed ;
                    possible_key = i ;
                }
                break ;
            case MaybePressed:
                if (i == possible_key) {
                    state = Pressed ;
                    confirmed_key = i ;
                }
                else {
                    state = NoPressed ;
                }
                break ;
            case Pressed:
                if (i != confirmed_key) {
                    state = MaybeNoPressed ;
                }
                break ;
            case MaybeNoPressed:
                if (i != confirmed_key) {
                    state = NoPressed ;
                    confirmed_key = -1 ;
                }
                else {
                    state = Pressed ;
                }
                break ;
        }

        // Overwrite i with confirmed key,
        // To avoid changes to original code
        i = confirmed_key ;


        if(i != -1 && i != fi) {
            // Set beep state to on
            if (i == 10){
                mode = RECORD;
                queue_record_length = 0;
            }else if (i == 11){
                mode = PLAY;
                copy_queue(queue_record, queue_play);
                queue_play_length = queue_record_length;
                beep_state = BEEP_ON;
                printf("\n%x", queue_record) ;
            }else if (mode == PLAY){
                queue_play[0] = i;
                queue_play_length = 1;
                beep_state = BEEP_ON ;
            }else if (mode == RECORD){
                queue_record[queue_record_length++] = i;
                queue_play[0] = i;
                queue_play_length = 1;
                beep_state = BEEP_ON ;
            }


        } else {
            // Set beep state to off
            //beep_state = BEEP_OFF ;
        }

        // Write key to VGA
        if (i != prev_key) {
            prev_key = i ;
            fillRect(250, 20, 176, 30, RED); // red box
            sprintf(keytext, "%d", i) ;
            setCursor(250, 20) ;
            setTextSize(2) ;
            writeString(keytext) ;
        }

        // Print key to terminal
        printf("\n%d", i) ;

        // save previous state
        fi = i ;

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}


int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize the VGA screen
    initVGA() ;

    // Draw some filled rectangles
    fillRect(64, 0, 176, 50, BLUE); // blue box
    fillRect(250, 0, 176, 50, RED); // red box
    fillRect(435, 0, 176, 50, GREEN); // green box

    // Write some text
    setTextColor(WHITE) ;
    setCursor(65, 0) ;
    setTextSize(1) ;
    writeString("Raspberry Pi Pico") ;
    setCursor(65, 10) ;
    writeString("Keypad demo") ;
    setCursor(65, 20) ;
    writeString("Hunter Adams") ;
    setCursor(65, 30) ;
    writeString("vha3@cornell.edu") ;
    setCursor(250, 0) ;
    setTextSize(2) ;
    writeString("Key Pressed:") ;

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;
    

    // Map MEASURE_ISR_GPIO to GPIO port, make it low
    gpio_init(MEASURE_ISR_GPIO) ;
    gpio_set_dir(MEASURE_ISR_GPIO, GPIO_OUT) ;
    gpio_put(MEASURE_ISR_GPIO, 0) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Create a repeating timer that calls 
    // repeating_timer_callback (defaults core 0)
    struct repeating_timer timer_core_0;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    add_repeating_timer_us(-25, 
        repeating_timer_callback_core_0, NULL, &timer_core_0);

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}
