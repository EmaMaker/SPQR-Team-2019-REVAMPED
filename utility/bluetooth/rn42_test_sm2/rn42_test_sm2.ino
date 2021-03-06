/*
 * EmaMaker - Fri 15/01/2021 - 17:20
 * A little test for the Trigger Mode (SM, 2) of our RN-42 BT modules
 * As stated in the Datasheet, the module will attempt a connection with the other module and keep it as long as it receives packages through Serial
 * To use this, will let one module bombard the other of packets to let them attempt a connection (When trying to connect, the module will not accept another incoming connection, 
 * so the robot will have to be turned on one after the other, waiting for the packet bombarding to finish). Once connected, the modules will keep talking to each other as long as they receive
 * packets from each other. When one robot (module) turns off, the other will stop receiving packets, so it will stop sending other packets, and will disconnect.
 * 
 * Configuration commands used in this example
 * SM,2 - Trigger Mode
 * SR,000666DCxxxx - The MAC Address of the other module
 * ST,3 - Timeout of three seconds. After this time not receiving packets from the Serial port, the module will disconnect from the other
 * SU,19200 - Baudrate of the UART with the uC. Don't know if this is really needed, but i found it like this and didn't feel like changing :D
*/

#define BT1 Serial3
#define BT2 Serial1

//I put some test LEDs on the breadboard to show when the code thinks the BTs are connected to each other
#define LED1 4
#define LED2 5

boolean mate1 = false;
boolean mate2 = false;

void setup(){
  Serial.begin(115200);
  pinMode(12, INPUT);
  pinMode(10, INPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  
  //Wait a bit for the modules to come up
  delay(2000);

  //Init connections
  BT1.begin(19200);
  BT2.begin(19200);

  //Wait again, just in case
  delay(1500);
}

unsigned long t = 0;

void loop(){

  if(digitalRead(12)) bombard(0);
  if(digitalRead(10)) bombard(1);
  
  //If receiving packages from BT, send something back
  //As long as the uC sends stuff in UART a module, it will stay connected to the other one
  //Use a little timeout with millis to not spam the Serial port, otherwise we will have a delay 
  if(millis() - t > 300){
    if(BT2.available()) {
      BT2.write("A");
      Serial.print("BT2: ");
      Serial.println(BT2.read());
      mate2 = true;
    } else     mate2 = false;
    
    if(BT1.available()) {
      BT1.write("A");
      Serial.print("BT1: ");
      Serial.println(BT1.read());
      mate1 = true;
    }else 
    mate1 = false;
    t= millis();
    

  digitalWrite(LED1, mate1);
  digitalWrite(LED2, mate2);
  }  

}

//Simulate the turning on of a robot,  this should happen when the robot is turning on
//Bombard the other one of packets, making them connect (SM, 2 - Trigger Mode)
void bombard(int a){
  for(int i = 0; i < 20; i++){
    Serial.print("Bombarding from");
    Serial.println(a);
  
    if (a == 0) BT1.print("*");
    else BT2.print("*");
    
    delay(100);
  }
}
