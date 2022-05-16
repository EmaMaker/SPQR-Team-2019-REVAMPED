 #define GAMES_CPP

/* #include "sensors/linesys_2019.h" */
#include "systems/lines/linesys_camera.h"
#include "systems/systems.h"
#include "systems/position/positionsys_zone.h"
#include "systems/position/positionsys_camera.h"
#include "strategy_roles/games.h"

void initGames(){
    vector<DataSource*> lIn = { new DataSource(S1I, true), new DataSource(S2I, true), new DataSource(S3I, true), new DataSource(S4I, true) };
    vector<DataSource*> lOut = { new DataSource(S1O, true), new DataSource(S2O, true), new DataSource(S3O, true), new DataSource(S4O, true) };

    striker = new Striker(new LineSysCamera(lIn, lOut), new PositionSysCamera());
    keeper = new Keeper(new LineSysCamera(lOut, lOut), new PositionSysCamera());
}