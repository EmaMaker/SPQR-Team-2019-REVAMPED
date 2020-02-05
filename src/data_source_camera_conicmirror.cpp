#include "data_source_camera_conicmirror.h"
#include "sensors.h"

DataSourceCameraConic::DataSourceCameraConic(HardwareSerial* ser_, int baud) : DataSource(ser_, baud){
  true_xb = 0;
  true_yb = 0;
  true_xy = 0;
  true_yy = 0;
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
}

void DataSourceCameraConic :: readSensor(){
  while(ser->available() > 0){
    value = (int)ser->read();
    //Serial.println(value);
    if(value==startp){
      start=true;
      count=0;
    }
    else if(value==endp){
      data_received=false;
      if(count=4 && start==true) {
        data_received=true;

        true_xb = xb-50;
        true_yb = 50-yb;
        true_xy = xy-50;
        true_yy = 50-yy;

        //Remap from [0,100] to [-50, +49] to correctly compute angles and distances and calculate them
        yAngle = -90-(atan2(true_yy, true_xy) * 180 /3.14);
        bAngle = -90-(atan2(true_yb, true_xb)* 180 /3.14);
        //Now cast angles to [0, 359] domain angle flip them
        yAngle = (yAngle+360)%360;
        bAngle = (bAngle+360)%360;

        int angleFix = compass->getValue() > 180 ? compass->getValue() - 360 : compass->getValue();

        //Fixes with IMU
        yAngleFix = ((int) ((yAngle + angleFix*0.8)) + 360) % 360 ;
        bAngleFix = ((int) ((bAngle + angleFix*0.8)) + 360) % 360;

        yDist = sqrt( (true_yy-50)*(true_yy-50) + (50-true_xy)*(50-true_xy) );
        bDist = sqrt( (true_yb-50)*(true_yb-50) + (50-true_xb)*(50-true_xb) );
      }
      end=true;
      start=false;
    }else{
      if(start==true){
        if (count==0) xb=value;
        else if (count==1) yb=value;
        else if (count==2) xy=value;
        else if (count==3) yy=value;
        count++;
      }
    }   
  }
}


int DataSourceCameraConic::getValueAtk(bool fixed){
  if(fixed) return goalOrientation == HIGH ? yAngleFix : bAngleFix;
  else return goalOrientation == HIGH ? yAngle : bAngle;
}
int DataSourceCameraConic::getValueDef(bool fixed){
  if(fixed) return goalOrientation == LOW ? yAngleFix : bAngleFix;
  else return goalOrientation == LOW ? yAngle : bAngle;
}

void DataSourceCameraConic::test(){
  goalOrientation = digitalRead(SWITCH_SX);     //se HIGH attacco gialla, difendo blu
  update();
    DEBUG.print("Blue: ");
    DEBUG.print(bAngle);
    DEBUG.print(" | ");
    DEBUG.print(bAngleFix);
    DEBUG.print(" | ");
    DEBUG.println(bDist);
    DEBUG.println(" --- ");

    DEBUG.print("Yellow: ");
    DEBUG.print(yAngle);
    DEBUG.print(" | ");
    DEBUG.print(yAngleFix); 
    DEBUG.print(" | ");
    DEBUG.println(yDist);
    DEBUG.println("---------------");
    DEBUG.print("Data: ");
    DEBUG.print(true_xb);
    DEBUG.print("|");
    DEBUG.print(true_yb);
    DEBUG.print("|");
    DEBUG.print(true_xy);
    DEBUG.print("|");
    DEBUG.println(true_yy); 
    DEBUG.println("---------------");
    delay(150);
}