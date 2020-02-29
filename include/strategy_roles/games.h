#pragma once

#ifdef GAMES_CPP
#define g_extr 
#else
#define g_extr extern
#endif

#include <Arduino.h>
#include "strategy_roles/game.h"
#include "strategy_roles/goalie.h"
#include "strategy_roles/keeper.h"

void initGames();

g_extr Game* goalie;
g_extr Game* keeper;