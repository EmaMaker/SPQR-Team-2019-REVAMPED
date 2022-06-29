#include "systems/lines/linesys_camera.h"
#include "behaviour_control/status_vector.h"
#include "sensors/sensors.h"
#include "strategy_roles/games.h"
#include "systems/position/positionsys_camera.h"

LineSysCamera::LineSysCamera() {}
LineSysCamera::LineSysCamera(vector<DataSource *> in_, vector<DataSource *> out_)
{
    this->in = in_;
    this->out = out_;

    fboundsX = false;
    fboundsY = false;
    slow = false;

    linesensOldX = 0;
    linesensOldY = 0;

    tookLine = false;

    for (int i = 0; i < 4; i++)
    {
        linetriggerI[i] = 0;
        linetriggerO[i] = 0;
    }

    exitTimer = 0;
    linesens = 0;
}

void LineSysCamera ::update()
{
  CURRENT_INPUT_WRITE.lineByte = 0;
    inV = 0;
    outV = 0;
    tookLine = false;

    for (DataSource *d : in)
        d->readSensor();
    for (DataSource *d : out)
        d->readSensor();

    for (auto it = in.begin(); it != in.end(); it++)
    {
        i = it - in.begin();
        ds = *it;
        linetriggerI[i] = ds->getValue() > LINE_THRESH_CAM;
        CURRENT_INPUT_WRITE.lineByte |= linetriggerI[i] << i;
    }
    for (auto it = out.begin(); it != out.end(); it++)
    {
        i = it - out.begin();
        ds = *it;
        linetriggerO[i] = ds->getValue() > LINE_THRESH_CAM;
        CURRENT_INPUT_WRITE.lineByte |= linetriggerO[i] << (4+i);
    }

    for (int i = 0; i < 4; i++)
    {
        inV = inV | (linetriggerI[i] << i);
        outV = outV | (linetriggerO[i] << i);
    }

    if (inV > 0 || outV > 0)
    {
        if (millis() - exitTimer > EXIT_TIME)
        {
            fboundsX = true;
            fboundsY = true;
        }
        exitTimer = millis();
    }

    linesens |= inV | outV;
    CURRENT_DATA_WRITE.lineActive = linesens;
    outOfBounds();
}

void LineSysCamera::outOfBounds()
{
    // digitalWriteFast(BUZZER, LOW);
    if (fboundsX == true)
    {
        if (linesens & 0x02)
            linesensOldX = 2;
        else if (linesens & 0x08)
            linesensOldX = 8;
        if (linesensOldX != 0) fboundsX = false;
    }
    if (fboundsY == true)
    {
        if (linesens & 0x01)
            linesensOldY = 1;
        else if (linesens & 0x04)
            linesensOldY = 4;
        if (linesensOldY != 0) fboundsY = false;
    }

    if (millis() - exitTimer < EXIT_TIME)
    {
        CURRENT_DATA_WRITE.game->ps->goCenter();
        tookLine = true;
        tone(BUZZER, 220.00, 250);
    }
    else
    {
        // drive->canUnlock = true;
        linesens = 0;
        linesensOldY = 0;
        linesensOldX = 0;
    }
}

void LineSysCamera::test()
{
    update();
    DEBUG.print("Byte Read: ");
    DEBUG.println(CURRENT_INPUT_READ.lineByte, BIN);
    DEBUG.print("In: ");
    for (DataSource *d : in)
    {
        d->update();
        DEBUG.print(d->getValue());
        DEBUG.print(" | ");
    }
    DEBUG.print(" |---| ");
    DEBUG.print("Out: ");
    for (DataSource *d : out)
    {
        d->update();
        DEBUG.print(d->getValue());
        DEBUG.print(" | ");
    }
    DEBUG.println();
    for (int i = 0; i < 4; i++)
    {
        DEBUG.print(linetriggerI[i]);
        DEBUG.print(" | ");
        DEBUG.println(linetriggerO[i]);
    }

    DEBUG.println(inV);
    DEBUG.println(outV);
    DEBUG.println();
}