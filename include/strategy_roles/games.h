#pragma once

#ifdef GAMES_CPP
#define g_extr 
#else
#define g_extr extern
#endif

#include <Arduino.h>
#include "strategy_roles/game.h"
#include "strategy_roles/striker.h"
#include "strategy_roles/keeper.h"

void initGames();

g_extr Game* striker;
g_extr Game* striker_roller;
g_extr Game* precision_shooter;
g_extr Game* pass_and_shoot;
g_extr Game* keeper;

g_extr Game* tc1;
g_extr Game* tc2;
g_extr Game* st_tc1;
g_extr Game* st_tc3;
g_extr Game* tc3_1;
g_extr Game* tc3_2;