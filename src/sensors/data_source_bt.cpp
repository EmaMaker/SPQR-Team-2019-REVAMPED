#include "sensors/data_source_bt.h"

DataSourceBT :: DataSourceBT(HardwareSerial* ser_, int baud) : DataSource(ser_, baud){
    bt_timer = millis();
    bt_bombarded = false;
    can_send = false;

    comrade = false;

    last_received = 0;
    t = 0;

    tosend = 'A';
    received = '0';

    connect();
}

void DataSourceBT :: connect(){
    Serial3.print("$");
    Serial3.print("$");
    Serial3.print("$");
    delay(100);
    Serial3.println("C");
}

// void DataSourceBT :: reconnect(){
//     if(!comrade){
//     if(!b){
//       Serial3.print("$");
//       Serial3.print("$");
//       Serial3.print("$");
//     }else{
//       Serial3.println("C");
//     }

//     b = !b;
//   }else{
//     Serial3.println("---");
//   }
//   if(millis() - last_received > 2000) 
//   comrade = false;
// }

void DataSourceBT::send(){
  if(millis() - t >= 250 && can_send ){
    Serial1.print(tosend);
  }
}

void DataSourceBT::update(){
  // if(!bt_bombarded && can_bombard) connect();
  receive();
  send();
  if(comrade)Serial2.write(0b00000100);
}

void DataSourceBT :: test(){
    if (DEBUG.available()) {
    DEBUG.println((char)DEBUG.read());
    }
    if (Serial3.available()) {
    Serial3.write((char)Serial3.read());
  }
}