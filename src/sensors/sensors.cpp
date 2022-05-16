#define SENSORS_CPP
#include "sensors/sensors.h"

void initSensors(){
    pinMode(SWITCH_1, INPUT);
    pinMode(SWITCH_2, INPUT);

    pinMode(LED_R, OUTPUT);
    pinMode(LED_Y, OUTPUT);
    pinMode(LED_G, OUTPUT);
  
    drive = new DriveController(new Motor(12, 11, 4, 55), new  Motor(25, 24, 5, 135), new Motor(27, 26, 2, 225), new Motor(29, 28, 3, 305));
    // tone(BUZZER, 270, 250);
    // delay(350);
    compass = new DataSourceBNO055();
    // tone(BUZZER, 275, 250);
    // delay(350);
    ball = new DataSourceBall(&BALL_32U4, 57600);
    // tone(BUZZER, 280, 250);
    // delay(350);
    camera = new DataSourceCameraConic(&Serial3, 19200);
    // tone(BUZZER, 285, 250);
    // delay(350);
    bt = new DataSourceBT(&Serial1, 9600);
    roller = new Roller(30, 31, 1000, 2000, 500);
    ballPresence = new DataSourceBallPresence(A13, true);
 }

void updateSensors(){
    role = digitalRead(SWITCH_DX);
    camera->old_goalOrientation = camera ->goalOrientation;
    camera->goalOrientation = digitalRead(SWITCH_SX);
    robot_indentifier = digitalRead(SWITCH_ID);
    
    compass->update();
    ball->update();
    ballPresence->update();
    camera->update();

    bt->update();

    roller->update();
}