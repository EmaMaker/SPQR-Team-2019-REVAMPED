 #define GAMES_CPP

/* #include "sensors/linesys_2019.h" */
#include "systems/lines/linesys_camera.h"
#include "systems/systems.h"
#include "systems/position/positionsys_zone.h"
#include "systems/position/positionsys_camera.h"
#include "strategy_roles/games.h"

void initGames(){
    std::vector<DataSource*> lIn;
    lIn.push_back(new DataSource(S1I, true));
    lIn.push_back(new DataSource(S2I, true));
    lIn.push_back(new DataSource(S3I, true));
    lIn.push_back(new DataSource(S4I, true));

    std::vector<DataSource*> lOut;
    lOut.push_back(new DataSource(S1O, true));
    lOut.push_back(new DataSource(S2O, true));
    lOut.push_back(new DataSource(S3O, true));
    lOut.push_back(new DataSource(S4O, true));

    striker = new Striker(new LineSysCamera(lIn, lOut), new PositionSysCamera());
    keeper = new Keeper(new LineSystemEmpty(), new PositionSystemEmpty());
}