#include "behaviour_control/status_vector.h"
#include "systems/position/positionsys_camera.h"
#include "sensors/sensors.h"
#include "sensors/data_source_ball.h"
#include "strategy_roles/striker.h"
#include "vars.h"
#include "math.h"


Striker::Striker() : Game()
{
  init();
}

Striker::Striker(LineSystem *ls_, PositionSystem *ps_) : Game(ls_, ps_)
{
  init();
}

void Striker::init()
{
  atk_speed = 0;
  atk_direction = 0;
  atk_tilt = 0;
  ball_angle_filter = 0;

  gotta_tilt = false;
  ball_filter = new ComplementaryFilter(1);
}

void Striker::realPlay()
{
  ball_angle_filter = ball_filter->calculate(CURRENT_DATA_READ.ballAngle);

  if (CURRENT_DATA_READ.ballSeen)
    this->striker();
  else{
    ps->goCenter();
  }
}

float ctilt = 0;
unsigned long ttilt = 0;

void Striker::striker(){
  if(CURRENT_DATA_READ.ballDistance >= 125){
    drive->prepareDrive(ball_angle_filter > 180 ? ball_angle_filter*0.96 : ball_angle_filter*1.04, MAX_VEL_3QUARTERS, 0);

  }else
{
  //seguo palla
  int ball_degrees2, dir, ball_deg = ball_angle_filter, plusang = STRIKER_PLUSANG;
  
  if(ball_deg >= 344 || ball_deg <= 16) plusang = STRIKER_PLUSANG_VISIONCONE;            //se ho la palla in un range di +-20 davanti, diminuisco di 20 il plus
  if(ball_deg > 180) ball_degrees2 = ball_deg - 360;            //ragiono in +180 -180  
  else ball_degrees2 = ball_deg;

  if(ball_degrees2 > 0) dir = ball_deg + plusang;               //se sto nel quadrante positivo aggiungo
  else dir = ball_deg - plusang;                                //se sto nel negativo sottraggo

  dir = (dir + 360) % 360;
  // drive->prepareDrive(dir, MAX_VEL_HALF, tilt());
  drive->prepareDrive(dir, MAX_VEL_3QUARTERS, CURRENT_DATA_READ.ballAngle <= 90 || CURRENT_DATA_READ.ballAngle >= 270 ? CURRENT_DATA_READ.angleAtkFix : 0);

  // if(ball->isInFront() && roller->roller_armed) roller->speed(ROLLER_DEFAULT_SPEED);
  // else roller->speed(roller->MIN);
  }
  
}

int Striker::tilt() {
  if (ball->isInMouth() /* || (ball->isInMouthMaxDistance() && gotta_tilt)*/ ) gotta_tilt = true;
  else gotta_tilt = false;

  if(!gotta_tilt || !CURRENT_DATA_READ.atkSeen) {
    atk_tilt *= 0.8;
    if(atk_tilt <= 10) atk_tilt = 0;
  }else{
    atk_tilt = roller->roller_armed ? CURRENT_DATA_READ.angleAtkFix : constrain(CURRENT_DATA_READ.angleAtkFix, -45, 45);
  }

  return atk_tilt;
}