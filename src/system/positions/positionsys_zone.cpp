#include "behaviour_control/status_vector.h"
#include "systems/position/positionsys_zone.h"
#include "sensors/sensors.h"
#include "vars.h"

PositionSysZone::PositionSysZone(){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
                zone[i][j] = 0;
        }
    }
    zoneIndex = 0;
    camA = 0;
    camD = 0;
    ao = 0;
    p = 4;
}

void PositionSysZone::PositionSysZone::update() {
    decreaseAll(ZONE_LOOP_DECREASE_VALUE);

    readPhyZone();
    //calculates guessed_x and guessed_y and zoneIndex
    //zoneIndex is just 2D to 1D of the guessed x and y
    //(y_position * width + x_position)

    int top = 0;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(zone[i][j] > top){
                guessed_x = i;
                guessed_y = j;
                top = zone[i][j];
            }
        }
    }
    zoneIndex = guessed_y * 3 + guessed_x;
}

int PositionSysZone::getValue(){
    return zoneIndex;
}

void PositionSysZone::PositionSysZone::test() {
    //outpus the matrix
    if (millis() - ao >= 500) {
        DEBUG.println("------");
        for (int i = 0; i < 4; i++) {
        DEBUG.print("US: ");
        usCtrl->test(); 
        DEBUG.print(" | ");
    }
    DEBUG.println();
    testPhyZone();
    testLogicZone();
    compass->test();

    // if (comrade){
    //   DEBUG.print("FriendZone: ");
    //   DEBUG.println(friendZone);
    // }
    DEBUG.println("------");
    ao = millis();
  }
}

int zone[3][3];

void PositionSysZone::increaseIndex(int i, int j, int ammount){
    if(i < 3 && j < 3){
      zone[i][j] += ammount;
      zone[i][j] = constrain(zone[i][j], 0, ZONE_MAX_VALUE);
    }
}

void PositionSysZone::decreaseIndex(int i, int j, int ammount){
  increaseIndex(i, j, -ammount);
}

void PositionSysZone::increaseRow(int i, int ammount){
    if(i < 3){
        for(int a = 0; a < 3; a++){
            increaseIndex(a, i, ammount);
        }
    }
}

void PositionSysZone::decreaseRow(int i, int ammount){
  increaseRow(i, -ammount);
}

void PositionSysZone::increaseCol(int i, int ammount){
    if(i < 3){
        for(int a = 0; a < 3; a++){
            increaseIndex(i, a, ammount);
        }
    }
}

void PositionSysZone::decreaseCol(int i, int ammount){
  increaseCol(i, -ammount);
}

void PositionSysZone::increaseColWithLimit(int i, int ammount){
  if(zone[i][0] + ammount < ZONE_MAX_VALUE && zone[i][1] + ammount < ZONE_MAX_VALUE && zone[i][2] + ammount < ZONE_MAX_VALUE){
    increaseCol(i, ammount);
  }
}

void PositionSysZone::increaseRowWithLimit(int i, int ammount){
  if(zone[0][1] + ammount < ZONE_MAX_VALUE && zone[1][i] + ammount < ZONE_MAX_VALUE && zone[2][i] + ammount < ZONE_MAX_VALUE){
    increaseRow(i, ammount);
  }
}

void PositionSysZone::decreaseColWithLimit(int i, int ammount){
  if(zone[i][0] - ammount >= 0 && zone[i][1] - ammount >= 0 && zone[i][2] - ammount >= 0){
    decreaseCol(i, ammount);
  }
}

void PositionSysZone::decreaseRowWithLimit(int i, int ammount){
  if(zone[0][i] - ammount >= 0 && zone[1][i] - ammount >= 0 && zone[2][i] - ammount >= 0){
    decreaseRow(i, ammount);
  }
}

void PositionSysZone::increaseAll(int val){
    //decrease all
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            increaseIndex(i, j, val);
        }
    }
}

void PositionSysZone::decreaseAll(int val){
  increaseAll(-val);
}


void PositionSysZone::readPhyZone(){
  phyZoneUS();
  phyZoneCam();
  phyZoneLines();
  // phyZoneDirection();
}

//old WhereAmI. Renamed to be coerent. Now also adds to the logic zone
void PositionSysZone::phyZoneUS(){
  // decide la posizione in orizzontale e verticale
  // Aggiorna i flag :  good_field_x  good_field_y      non utilizzata da altre
  // routines
  //                    goal_zone                       non utilizzata da altre
  //                    routines
  // Aggiorna le variabili:
  //                    status_x (con valori CENTER EAST  WEST 255  = non lo so)
  //                    status_y (con valori CENTER NORTH SOUTH   255  = non lo so)

  int Lx_mis; // larghezza totale stimata dalle misure
  int Ly_mis; // lunghezza totale stimata dalle misure
  int Ly_min; // Limite inferiore con cui confrontare la misura y
  int Ly_max; // Limite inferiore con cui confrontare la misura y
  int Dy;     // Limite per decidere NORTH SOUTH in funzione della posizione
              // orizzontale

  old_status_x = status_x;
  old_status_y = status_y;
  good_field_x = false; // non é buona x
  good_field_y = false; // non é buona y
  goal_zone = false;    // non sono davanti alla porta avversaria

  if (role == HIGH)
    DxF = DxF_Atk;
  else
    DxF = DxF_Def;

  Lx_mis = CURRENT_DATA_READ.USdx + CURRENT_DATA_READ.USsx + robot; // larghezza totale stimata
  Ly_mis = CURRENT_DATA_READ.USfr + CURRENT_DATA_READ.USrr + robot; // lunghezza totale stimata

  // controllo orizzontale
  if ((Lx_mis < Lx_max) && (Lx_mis > Lx_min) && (CURRENT_DATA_READ.USdx > 25) && (CURRENT_DATA_READ.USsx > 25)) {
    // se la misura orizzontale é accettabile
    good_field_x = true;
    status_x = CENTER;
    if (CURRENT_DATA_READ.USdx < DxF) // robot é vicino al bordo dEASTro
      status_x = EAST;
    if (CURRENT_DATA_READ.USsx < DxF) // robot é vicino al bordo sinistro
      status_x = WEST;

    if (status_x == CENTER) {
      // imposto limiti di controllo lunghezza verticale tra le porte
      Ly_min = LyP_min;
      Ly_max = LyP_max;
      Dy = DyP;
    } else {
      // imposto limiti di controllo lunghezza verticale in fascia
      Ly_min = LyF_min;
      Ly_max = LyF_max;
      Dy = DyF;
    }
  } else {
    // la misura non é pulita per la presenza di un ostacolo
    if ((CURRENT_DATA_READ.USdx >= (DxF + 10)) || (CURRENT_DATA_READ.USsx >= (DxF + 10))) {
      // se ho abbastanza spazio a dEASTra o a sinistra
      // devo stare per forza al cento
      status_x = CENTER;
      // imposto limiti di controllo lunghezza verticale tra le porte
      Ly_min = LyP_min;
      Ly_max = LyP_max;
      Dy = DyP;
    } else {
      status_x = 255;
      // non so la coordinata x
      // imposto i limiti di controllo verticale in base alla posizione
      // orizzontale precedente
      if (old_status_x == CENTER) {
        // controlla la posizione precedente per decidere limiti di controllo y
        // imposto limiti di controllo lunghezza verticale tra le porte
        Ly_min = LyP_min;
        Ly_max = LyP_max;
        Dy = DyP;
      } else {
        // imposto limiti di controllo lunghezza verticale in fascia anche per x
        // incognita
        Ly_min = LyF_min;
        Ly_max = LyF_max;
        Dy = DyF;
      }
    }
  }
  // controllo verticale
  if ((Ly_mis < Ly_max) && (Ly_mis > Ly_min)) {
    // se la misura verticale é accettabile
    good_field_y = true;
    status_y = CENTER;
    if (CURRENT_DATA_READ.USfr < Dy) {
      status_y = NORTH; // robot é vicino alla porta avversaria
      if (Dy == DyP)
        goal_zone = true; //  davanti alla porta in zona goal
    }
    if (CURRENT_DATA_READ.USrr < Dy)
      status_y = SOUTH; // robot é vicino alla propria porta
  } else {
    // la misura non é pulita per la presenza di un ostacolo
    status_y = 255; // non so la coordinata y
    if (CURRENT_DATA_READ.USfr >= (Dy + 0))
      status_y = CENTER; // ma se ho abbastanza spazio dietro o avanti
    if (CURRENT_DATA_READ.USrr >= (Dy + 0))
      status_y = CENTER; //  e'probabile che stia al CENTER
  }

  //now operates on the matrix
  if (status_x == 255 && status_y != 255) {
      increaseRow(status_y, ZONE_US_UNKNOWN_INCREASE_VALUE);
  } else if (status_x != 255 && status_y == 255) {
      increaseCol(status_x, ZONE_US_UNKNOWN_INCREASE_VALUE);
  } else {
      increaseIndex(status_x, status_y, ZONE_US_INDEX_INCREASE_VALUE);
  }
}

void PositionSysZone::phyZoneCam(){

  //IMU-fixed attack angle
  camA = CURRENT_DATA_READ.atkGAngle_fix;
  //IMU-fixed defence angle
  camD = CURRENT_DATA_READ.defGAngle_fix;

  //Negative angle means that the robot is positioned to the right of the goalpost
  //Positive angle means that the robot is positioned to the left of the goalpost

  p = 4;

  if(abs(diff(camA, camD)) <= ZONE_CAM_CENTER_RANGE){
    //at center row, you can consider both camA and camD
      p = 1;
  }else if(camA > camD){
      p = 0;
  }else if(camD > camA){
      p = 2;
  }

  increaseColWithLimit(p, ZONE_CAM_INCREASE_VALUE);

  calcPhyZoneCam = false;
}



void PositionSysZone::phyZoneLines(){
//   //ZONE_LINES_ERROR_VALUE is a random error code not used in line exit direction calculations
//   if(lineSensByteBak != ZONE_LINES_ERROR_VALUE){ 
//     switch(lineSensByteBak) {
//       case 1:         //NORTH
//         increaseRow(0, ZONE_LINES_INCREASE_VALUE);
//         decreaseRow(1, ZONE_LINES_INCREASE_VALUE);
//         decreaseRow(2, ZONE_LINES_INCREASE_VALUE);
//       break;

//       case 2:         //EAST
//         decreaseCol(0, ZONE_LINES_INCREASE_VALUE);
//         decreaseCol(1, ZONE_LINES_INCREASE_VALUE);
//         increaseCol(2, ZONE_LINES_INCREASE_VALUE);
//       break;

//       case 4:         //SOUTH
//         decreaseRow(0, ZONE_LINES_INCREASE_VALUE);
//         decreaseRow(1, ZONE_LINES_INCREASE_VALUE);
//         decreaseRow(2, ZONE_LINES_INCREASE_VALUE);
//       break;

//       case 8:         //WEST
//         increaseCol(0, ZONE_LINES_INCREASE_VALUE);
//         decreaseCol(1, ZONE_LINES_INCREASE_VALUE);
//         decreaseCol(2, ZONE_LINES_INCREASE_VALUE);
//       break;

//       case 3:
//         decreaseAll(ZONE_LINES_INCREASE_VALUE);
//         increaseIndex(0, 2, 2*ZONE_LINES_INCREASE_VALUE);
//       break;

//       case 6:
//         decreaseAll(ZONE_LINES_INCREASE_VALUE);
//         increaseIndex(2, 2, 2*ZONE_LINES_INCREASE_VALUE);
//       break;

//       case 9:
//         decreaseAll(ZONE_LINES_INCREASE_VALUE);
//         increaseIndex(0, 0, 2*ZONE_LINES_INCREASE_VALUE);
//       break;
        
//       default:
//       break;
//     }
//     //Last thing to do, sets the var to an error code, so next time it will be called will be because of the outOfBounds function being called
//     lineSensByteBak = ZONE_LINES_ERROR_VALUE;
//   }
}


void PositionSysZone::testPhyZone(){
    updateSensors();

    readPhyZone();
    DEBUG.print("Measured US location:\t");
    DEBUG.print(status_x);
    DEBUG.print(" | ");
    DEBUG.println(status_y);
    DEBUG.print("Measured Cam Column (4 is error):\t");
    DEBUG.println(p);
}

void PositionSysZone::testLogicZone(){
    update();
    DEBUG.println("-----------------");

    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
        DEBUG.print(zone[i][j]);
        DEBUG.print(" | ");
        }
        DEBUG.println();
    }
    DEBUG.println("-----------------");
    DEBUG.print("Guessed location:\t");
    DEBUG.print(guessed_x);
    DEBUG.print(" | ");
    DEBUG.println(guessed_y);
    DEBUG.print("Zone Index: ");
    DEBUG.println(zoneIndex);
}


void PositionSysZone::goCenter() {
  if (zoneIndex == 8)
    drive->prepareDrive(330, GOCENTER_VEL);
  if (zoneIndex == 7)
    drive->prepareDrive(0, GOCENTER_VEL);
  if (zoneIndex == 6)
    drive->prepareDrive(45, GOCENTER_VEL);
  if (zoneIndex == 5)
    drive->prepareDrive(270, GOCENTER_VEL);
  if (zoneIndex == 4)
    drive->prepareDrive(0, 0);
  if (zoneIndex == 3)
    drive->prepareDrive(90, GOCENTER_VEL);
  if (zoneIndex == 2)
    drive->prepareDrive(255, GOCENTER_VEL);
  if (zoneIndex == 1)
    drive->prepareDrive(180, GOCENTER_VEL);
  if (zoneIndex == 0)
    drive->prepareDrive(135, GOCENTER_VEL);
}


// int vel = 160;
// int usDist = 70;
// void PositionSysZone::centerGoalPost() {
//   x = 1;
//   y = 1;
//   int vel = 255;
//   if ((zoneIndex >= 0 && zoneIndex <= 2) || zoneIndex == 4) {
//     drive->prepareDrive(180, vel);
//   } else if (zoneIndex == 3 || zoneIndex == 6) {
//     drive->prepareDrive(90, vel);
//   } else if (zoneIndex == 5 || zoneIndex == 8) {
//     drive->prepareDrive(270, vel);
//   } else {
//     stop_menamoli = false;
//     if (CURRENT_DATA_READ.USrr >= 25)
//       drive->prepareDrive(180, vel);
//     else
//       drive->prepareDrive(0, 0);
//   }
// }


void PositionSysZone::AAANGOLO() {
  if((CURRENT_DATA_READ.USrr <= 45) && ((CURRENT_DATA_READ.USdx <= 50) || (CURRENT_DATA_READ.USsx <= 50))) drive->prepareDrive(0, 350, 0);
}

int PositionSysZone::diff(int a, int b){
	int diffB = abs(min(a, b) - max(a, b));
	int diffB1 = 360-diffB;
	int diff = min(diffB, diffB1);
  return diff;
}