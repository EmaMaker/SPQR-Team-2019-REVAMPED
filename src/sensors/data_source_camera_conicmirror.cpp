#include "behaviour_control/status_vector.h"
#include "sensors/data_source_camera_conicmirror.h"
#include "vars.h"

//Comment out to disable complementary filters on angles
#define CAMERA_CONIC_FILTER_POINTS

DataSourceCameraConic::DataSourceCameraConic(HardwareSerial *ser_, int baud) : DataSource(ser_, baud) {
  true_xb = 0;
  true_yb = 0;
  true_xy = 0;
  true_yy = 0;
  true_xb_fixed = 0;
  true_yb_fixed = 0;
  true_xy_fixed = 0;
  true_yy_fixed = 0;
  xb = 0;
  yb = 0;
  xy = 0;
  yy = 0;
  start = false;
  data_received = false;
  end = false;
  yAngle = 0;
  yAngleFix = 0;
  yDist = 0;
  bAngle = 0;
  bAngleFix = 0;
  bDist = 0;
  true_xb_fixed = 0;
  true_yb_fixed = 0;
  true_xy_fixed = 0;
  true_yy_fixed = 0;

  goalOrientation = 0;
  old_goalOrientation = 0;

  filter_yy = new ComplementaryFilter(FILTER_YY_COEFF);
  filter_xy = new ComplementaryFilter(FILTER_YX_COEFF);
  filter_yb = new ComplementaryFilter(FILTER_BY_COEFF);
  filter_xb = new ComplementaryFilter(FILTER_BX_COEFF);
  filter_yy_fix = new ComplementaryFilter(FILTER_YY_COEFF);
  filter_xy_fix = new ComplementaryFilter(FILTER_YX_COEFF);
  filter_yb_fix = new ComplementaryFilter(FILTER_BY_COEFF);
  filter_xb_fix = new ComplementaryFilter(FILTER_BX_COEFF);
}

void DataSourceCameraConic ::readSensor() {
  while (ser->available() > 0) {
    value = (int)ser->read();
    //Serial.println(value);
    if (value == startp) {
      start = true;
      count = 0;
    } else if (value == endp) {
      data_received = false;
      if ((count == 4) && (start == true)) {
        data_received = true;

        true_xb = xb;
        true_yb = yb;
        true_xy = xy;
        true_yy = yy;

        computeCoordsAngles();
      }
      end = true;
      start = false;
    } else{
      if (start == true)
      {
        if (count == 0)
          xb = value;
        else if (count == 1)
          yb = value;
        else if (count == 2)
          xy = value;
        else if (count == 3)
          yy = value;
        count++;
      }
    }
  }
}


void DataSourceCameraConic ::computeCoordsAngles() {
  CURRENT_DATA_WRITE.bSeen = true_xb != unkn && true_yb != unkn;
  CURRENT_DATA_WRITE.ySeen = true_xy != unkn && true_yy != unkn;

  //Where are the goals relative to the robot?
  //Remap from [0,100] to [-50, +50] to correctly compute angles and distances and calculate them, getting the original coords calculated by the camera
  true_xb = 50 - true_xb + CAMERA_TRANSLATION_X*0.5;
  true_yb = true_yb - 50 + CAMERA_TRANSLATION_Y*0.5;
  true_xy = 50 - true_xy + CAMERA_TRANSLATION_X*0.5;
  true_yy = true_yy - 50 + CAMERA_TRANSLATION_Y*0.5;

  #ifdef CAMERA_CONIC_FILTER_POINTS
  true_xb = filter_xb->calculate(true_xb);
  true_yb = filter_yb->calculate(true_yb);
  true_xy = filter_xy->calculate(true_xy);
  true_yy = filter_yy->calculate(true_yy);
  #endif

  //-90 + to account for phase shifting with goniometric circle
  yAngle = -90 + (atan2(true_yy, true_xy) * 180 / 3.14);
  bAngle = -90 + (atan2(true_yb, true_xb) * 180 / 3.14);
  //Now cast angles to [0, 359] domain angle flip them
  yAngle = (yAngle + 360) % 360;
  bAngle = (bAngle + 360) % 360;
  yDist = sqrt((true_yy) * (true_yy) + (true_xy) * (true_xy));
  bDist = sqrt((true_yb) * (true_yb) + (true_xb) * (true_xb));
  // int angleFix = CURRENT_DATA_READ.IMUAngle > 180 ? CURRENT_DATA_READ.IMUAngle - 360 : CURRENT_DATA_READ.IMUAngle;

  // //Fixes with IMU
  // yAngleFix = ((int)((yAngle + angleFix * 0.8)) + 360) % 360;
  // bAngleFix = ((int)((bAngle + angleFix * 0.8)) + 360) % 360;

  //Rotate the points of the goals on the cartesian plane to compensate the robot tilt
  int tmp = CURRENT_DATA_READ.IMUAngle > 180 ? CURRENT_DATA_READ.IMUAngle - 360 : CURRENT_DATA_READ.IMUAngle;
  //We are considering the angle being negative if going counterclockwise and positive is going clockwise.
  //The formula used below assumes the angle being positive in counterclockwise and negative in clockwise, so the angle must be multiplied by -1
  //A little explanation of the formula used here can be found on wikipedia: https://en.wikipedia.org/wiki/Rotation_of_axes
  float angleFix = -tmp*3.14/180;
  true_xb_fixed = (true_xb*(cos(angleFix))) + (true_yb*(sin(angleFix)));
  true_yb_fixed = (-true_xb*(sin(angleFix))) + (true_yb*(cos(angleFix)));
  true_xy_fixed = (true_xy*(cos(angleFix))) + (true_yy*(sin(angleFix)));
  true_yy_fixed = (-true_xy*(sin(angleFix))) + (true_yy*(cos(angleFix)));

  #ifdef CAMERA_CONIC_FILTER_POINTS
  true_xb_fixed = filter_xb_fix->calculate(true_xb_fixed);
  true_yb_fixed = filter_yb_fix->calculate(true_yb_fixed);
  true_xy_fixed = filter_xy_fix->calculate(true_xy_fixed);
  true_yy_fixed = filter_yy_fix->calculate(true_yy_fixed);
  #endif

  yAngleFix = -90 + (atan2(true_yy_fixed, true_xy_fixed) * 180 / 3.14);
  bAngleFix = -90 + (atan2(true_yb_fixed, true_xb_fixed) * 180 / 3.14);

  //Important: update status vector
  CURRENT_INPUT_WRITE.cameraByte = value;
  CURRENT_DATA_WRITE.cam_xb = true_xb;
  CURRENT_DATA_WRITE.cam_yb = true_yb;
  CURRENT_DATA_WRITE.cam_xy = true_xy;
  CURRENT_DATA_WRITE.cam_yy = true_yy;
  CURRENT_DATA_WRITE.cam_xb_fixed = true_xb_fixed;
  CURRENT_DATA_WRITE.cam_yb_fixed = true_yb_fixed;
  CURRENT_DATA_WRITE.cam_xy_fixed = true_xy_fixed;
  CURRENT_DATA_WRITE.cam_yy_fixed = true_yy_fixed;
  CURRENT_DATA_WRITE.yAngle = yAngle;
  CURRENT_DATA_WRITE.bAngle = bAngle;
  CURRENT_DATA_WRITE.yAngleFix = yAngleFix;
  CURRENT_DATA_WRITE.bAngleFix = bAngleFix;
  CURRENT_DATA_WRITE.yDist = yDist;
  CURRENT_DATA_WRITE.bDist = bDist;
  
  if (goalOrientation == HIGH) {
    CURRENT_DATA_WRITE.angleAtk = CURRENT_DATA_WRITE.yAngle;
    CURRENT_DATA_WRITE.angleAtkFix = CURRENT_DATA_WRITE.yAngleFix;
    CURRENT_DATA_WRITE.atkSeen = CURRENT_DATA_WRITE.ySeen;
    CURRENT_DATA_WRITE.distAtk = CURRENT_DATA_WRITE.yDist;
    
    CURRENT_DATA_WRITE.yAtk = CURRENT_DATA_WRITE.cam_yy;
    CURRENT_DATA_WRITE.yAtkFix = CURRENT_DATA_WRITE.cam_yy_fixed;
    CURRENT_DATA_WRITE.xAtk = CURRENT_DATA_WRITE.cam_xy;
    CURRENT_DATA_WRITE.xAtkFix = CURRENT_DATA_WRITE.cam_xy_fixed;

    CURRENT_DATA_WRITE.angleDef = CURRENT_DATA_WRITE.bAngle;
    CURRENT_DATA_WRITE.angleDefFix = CURRENT_DATA_WRITE.bAngleFix;
    CURRENT_DATA_WRITE.defSeen = CURRENT_DATA_WRITE.bSeen;
    CURRENT_DATA_WRITE.distDef = CURRENT_DATA_WRITE.bDist;

    CURRENT_DATA_WRITE.yDef = CURRENT_DATA_WRITE.cam_yb;
    CURRENT_DATA_WRITE.yDefFix = CURRENT_DATA_WRITE.cam_yb_fixed;
    CURRENT_DATA_WRITE.xDef = CURRENT_DATA_WRITE.cam_xb;
    CURRENT_DATA_WRITE.xDefFix = CURRENT_DATA_WRITE.cam_xb_fixed;
  } else {
    CURRENT_DATA_WRITE.angleAtk = CURRENT_DATA_WRITE.bAngle;
    CURRENT_DATA_WRITE.angleAtkFix = CURRENT_DATA_WRITE.bAngleFix;
    CURRENT_DATA_WRITE.atkSeen = CURRENT_DATA_WRITE.bSeen;
    CURRENT_DATA_WRITE.distAtk = CURRENT_DATA_WRITE.bDist;

    CURRENT_DATA_WRITE.yAtk = CURRENT_DATA_WRITE.cam_yb;
    CURRENT_DATA_WRITE.yAtkFix = CURRENT_DATA_WRITE.cam_yb_fixed;
    CURRENT_DATA_WRITE.xAtk = CURRENT_DATA_WRITE.cam_xb;
    CURRENT_DATA_WRITE.xAtkFix = CURRENT_DATA_WRITE.cam_xb_fixed;

    CURRENT_DATA_WRITE.angleDef = CURRENT_DATA_WRITE.yAngle;
    CURRENT_DATA_WRITE.angleDefFix = CURRENT_DATA_WRITE.yAngleFix;
    CURRENT_DATA_WRITE.defSeen = CURRENT_DATA_WRITE.ySeen;
    CURRENT_DATA_WRITE.distDef = CURRENT_DATA_WRITE.yDist;

    CURRENT_DATA_WRITE.yDef = CURRENT_DATA_WRITE.cam_yy;
    CURRENT_DATA_WRITE.yDefFix = CURRENT_DATA_WRITE.cam_yy_fixed;
    CURRENT_DATA_WRITE.xDef = CURRENT_DATA_WRITE.cam_xy;
    CURRENT_DATA_WRITE.xDefFix = CURRENT_DATA_WRITE.cam_xy_fixed;
  }

  // byte to_32u4 = 0;
  // to_32u4 |= (CURRENT_DATA_READ.ySeen);
  // to_32u4 |= (CURRENT_DATA_READ.bSeen) << 1;
  // BALL_32U4.write(to_32u4);
}

void DataSourceCameraConic::test(){
  update();
  DEBUG.print("Blue: Angle: ");
  DEBUG.print(bAngle);
  DEBUG.print(" | Fixed Angle: ");
  DEBUG.print(bAngleFix);
  DEBUG.print(" | Distance: ");
  DEBUG.print(bDist);
  DEBUG.print(" | Seen: ");
  DEBUG.println(CURRENT_DATA_READ.bSeen);
  DEBUG.print(" | Received coords: (");
  DEBUG.print(CURRENT_DATA_READ.cam_xb);
  DEBUG.print(",");
  DEBUG.print(CURRENT_DATA_READ.cam_yb);
  DEBUG.print(")");
  DEBUG.print(" | Fixed coords: (");
  DEBUG.print(CURRENT_DATA_READ.cam_xb_fixed);
  DEBUG.print(",");
  DEBUG.print(CURRENT_DATA_READ.cam_yb_fixed);
  DEBUG.println(")");
  DEBUG.println("----------------");
  DEBUG.print("Yellow: Angle: ");
  DEBUG.print(yAngle);
  DEBUG.print(" | Fixed Angle: ");
  DEBUG.print(yAngleFix);
  DEBUG.print(" | Distance: ");
  DEBUG.print(yDist);
  DEBUG.print(" | Seen: ");
  DEBUG.println(CURRENT_DATA_READ.ySeen);
  DEBUG.print(" | Received coords: (");
  DEBUG.print(CURRENT_DATA_READ.cam_xy);
  DEBUG.print(",");
  DEBUG.print(CURRENT_DATA_READ.cam_yy);
  DEBUG.print(")");
  DEBUG.print(" | Fixed coords: (");
  DEBUG.print(CURRENT_DATA_READ.cam_xy_fixed);
  DEBUG.print(",");
  DEBUG.print(CURRENT_DATA_READ.cam_yy_fixed);
  DEBUG.println(")");
  DEBUG.println("---------------");
  delay(150);
}
