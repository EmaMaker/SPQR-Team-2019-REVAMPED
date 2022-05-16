#pragma once
#define DEBUG Serial

#ifdef VARS
#define extr
#else
#define extr extern
#endif

#define GLOBAL_SPD_MULT 1.0

/*SWS and LEDS are to be tested and implemented in the code. 
the new setup is the one commented, leds have to be written 
in the 32U4 code*/

#define LED_R 20
#define LED_Y 17
#define LED_G 13

#define BUZZER 6
#define SWITCH_SX 39
#define SWITCH_DX 38
#define SWITCH_ID 33

#define ROLLER_INA 34
#define ROLLER_INB 35

#define BALL_32U4 Serial2

extr float sins[360], cosins[360];
