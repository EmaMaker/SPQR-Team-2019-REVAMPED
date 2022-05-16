#pragma once
#define DEBUG Serial

#ifdef VARS
#define extr
#else
#define extr extern
#endif

#define GLOBAL_SPD_MULT 1.0

#define LED_R 20
#define LED_Y 17
#define LED_G 13

#define BUZZER 30
#define SWITCH_SX 28
#define SWITCH_DX 29

extr float sins[360], cosins[360];
