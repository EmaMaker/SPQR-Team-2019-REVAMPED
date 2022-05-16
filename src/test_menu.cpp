#include "test_menu.h"
#include "sensors/data_source_ball.h"
#include "sensors/data_source_bno055.h"
#include "sensors/data_source_bt.h"
#include "sensors/data_source_camera_conicmirror.h"
#include "systems/lines/linesys_camera.h"
#include "sensors/sensors.h"
#include "motors_movement/motor.h"
#include "motors_movement/drivecontroller.h"
#include "systems/position/positionsys_camera.h"
#include "strategy_roles/game.h"
#include "strategy_roles/games.h"
#include "behaviour_control/data_source.h"
#include "behaviour_control/status_vector.h"

unsigned long timer = 0;

void TestMenu::testMenu()
{
    if (!(DEBUG.available() || flagtest))
        return;

    flagtest = true;

    if (DEBUG.available())
    {
        testNum = (char)DEBUG.read();

        DEBUG.print("You chose test: ");
        DEBUG.println(testNum);
    }

    if (flagtest)
    {

        drive->resetDrive();

        switch (testNum)
        {
        case '0':
            DEBUG.println("Exiting test menu, may the odds be in your favor c:");
            flagtest = false;
            DEBUG.flush();
            return;
            break;
        case '1':
            DEBUG.println("Ball Test");
            ball->test();
            delay(TEST_DELAY);
            break;
        case '2':
            DEBUG.println("IMU Test");
            compass->test();
            delay(TEST_DELAY);
            break;
        case '3':
            DEBUG.println("Motors Test. Lift up from ground and turn on S_MOT");
            drive->stopAll();
            drive->m1->test();
            drive->m2->test();
            drive->m3->test();
            drive->m4->test();
            flagtest = true;
            break;
        case '4':
            DEBUG.println("Pid recenter test");
            drive->drive(0, 0, 0);
            break;
        case '5':
            DEBUG.println("BT Test");
            bt->test();
            break;
        case '6':
            DEBUG.println("Camera Test");
            camera->test();
            delay(100);
            break;
        case '7':
            DEBUG.println("LineSensors Test");
            CURRENT_DATA_READ.game->ls->test();
            delay(200);
            break;
        case '8':
            DEBUG.println("Camera tilt Test");
            drive->resetDrive();
            drive->prepareDrive(0, 0, CURRENT_DATA_READ.angleAtkFix);
            break;
        case '9':
            DEBUG.println("Switches as seen from behind:");
            DEBUG.print("Right switch (role): ");
            DEBUG.println(role);
            DEBUG.print("Left switch (goalOrientation): ");
            DEBUG.println(camera->goalOrientation);
            DEBUG.print("Robot Identifier: ");
            delay(50);
            break;
        case 'p':
            if(millis() - timer > 150){
                timer = millis();
                DEBUG.println("Position on the field (camera)");
                DEBUG.print("X: ");
                DEBUG.print(CURRENT_DATA_READ.posx);
                DEBUG.print(" , Y: ");
                DEBUG.println(CURRENT_DATA_READ.posy);
            }
            break;
        case 'u':
            DEBUG.println("32u4 receive Test");
            while (Serial2.available())
                DEBUG.print((char)Serial2.read());
            break;
        case 'b':
            // ballPresence->test();
            break;
        case 'r':
            drive->stopAll();
            DEBUG.println("Roller at default speed");
            roller->speed(roller->MAX);
            break;
        case 's':
            DEBUG.println("32u4 send Test");
            DEBUG.println("Remember LED1 is not used by teensy");
            DEBUG.println("LED2");
            Serial2.write(0b00000001);
            delay(1500);

            DEBUG.println("LED3");
            Serial2.write(0b00000010);
            delay(1500);

            DEBUG.println("LED4");
            Serial2.write(0b00000100);

            delay(1500);
            Serial2.write(0);
            delay(1500);
            break;
        case 'h':
        default:
            DEBUG.println("Unknown command, here's a lil help for you :)");

            DEBUG.println("Test Menu: ");
            DEBUG.println("0)Exit test menu and start playing");
            DEBUG.println("1)Ball test");
            DEBUG.println("2)IMU test");
            DEBUG.println("3)Motors test");
            DEBUG.println("4)Recenter");
            DEBUG.println("5)BT test");
            DEBUG.println("6)Camera test");
            DEBUG.println("7)Line Sensors camera test");
            DEBUG.println("8)Camera tilt test");
            DEBUG.println("9)Switches test");
            DEBUG.println("u)Read Serial messages from 32u4");
            DEBUG.println("s)Send test to 32u4 status LEDs");
            DEBUG.println("r)Test roller");
            flagtest = false;
        }
    }
}