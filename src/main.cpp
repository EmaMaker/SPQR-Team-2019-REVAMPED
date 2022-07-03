#include <Arduino.h>

#define VARS

#include "behaviour_control/status_vector.h"
#include "sensors/sensors.h"
#include "strategy_roles/games.h"
#include "vars.h"
#include "test_menu.h"
#include "motors_movement/roller.h"

TestMenu* testmenu;

#define BALL_IN A13

bool striker_condition = false;
bool keeper_condition = false;

void setup() {
  tone(BUZZER, 220, 250);
  delay(750);
  DEBUG.begin(115200);
    
  for(int i = 0; i < 360; i++){
      sins[i] = (float) sin((i*3.14/180));
      cosins[i] =  (float) cos((i*3.14/180));
  }

  testmenu = new TestMenu();
  tone(BUZZER, 240, 250);
  initStatusVector();
  delay(250);

  tone(BUZZER, 260, 250);
  initSensors();
  delay(500);
  
  tone(BUZZER, 320, 250);
  initGames();
  delay(450);

  //Startup sound
  tone(BUZZER, 350.00, 250);

  drive->prepareDrive(0,0,0);
}

void loop() {
  updateSensors();
  drive->resetDrive();
  testmenu->testMenu();

  keeper->play(!role);
  striker->play(role || ((Keeper*)keeper)->shouldStrike);
  
  drive->drivePrepared();
  updateStatusVector();

}