#define VERSION 214

#define BAUD 115200
#define BUZZER_PIN 9
#define RECEIVER_PIN 2
#define MOTOR_PINS 4,5,6,7

#define THROTTLE 2
#define YAW 3
#define ROLL 1
#define PITCH 0
#define LASTCHANNEL 8

#define FRONT 0
#define RIGHT 1
#define LEFT 2
#define REAR 3
#define LASTMOTOR 4

#define MOTOR_LOW 2000
#define MOTOR_HIGH 4000
#define MOTOR_OFF 200
#define MOTOR_ON MOTOR_LOW + 200

#define RX_LOW 1700
#define RX_HIGH 2800


// FUNCTION: return the number of bytes currently free in RAM      
extern int  __bss_end; // used by freemem 
extern int  *__brkval; // used by freemem
int freemem(){
    int free_memory;
    if((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
}
