#include <Arduino.h>

#define VARS

#include "behaviour_control/status_vector.h"
#include "sensors/sensors.h"
#include "strategy_roles/games.h"
#include "vars.h"
#include "test_menu.h"
#include "motors_movement/roller.h"

void updateRoller();

TestMenu* testmenu;

bool striker_condition = false;
bool keeper_condition = false;

void setup() {
  tone(BUZZER, 220, 250);
  delay(1500);
  DEBUG.begin(115200);
    
  for(int i = 0; i < 360; i++){
      sins[i] = (float) sin((i*3.14/180));
      cosins[i] =  (float) cos((i*3.14/180));
  }

  testmenu = new TestMenu();
  tone(BUZZER, 240, 250);
  initStatusVector();
  tone(BUZZER, 260, 250);
  initSensors();
  tone(BUZZER, 320, 250);
  initGames();

  delay(500);

  drive->prepareDrive(0,0,0);

  //Startup sound
  tone(BUZZER, 350.00, 250);
}

void loop() {
  updateSensors();

  drive->resetDrive();
  
  striker_condition = role == HIGH || ((Keeper*)keeper)->shouldStrike;
  keeper_condition = role == LOW;

  striker->play(striker_condition);
  keeper->play(keeper_condition);
  // testmenu->testMenu();

  // Last thing to do: movement and update status vector
  drive->drivePrepared();  
  updateStatusVector();
}