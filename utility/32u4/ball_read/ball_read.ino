/**
  Sensor Mapping
    Sensor  Angle Pin Port
    S1      0     26  PD6
    S2            25  PD4
    S3            19  PD1
    S4            18  PD0
    S5      90    1   PE6
    S6            40  PF1
    S7            39  PF4
    S8            38  PF5
    S9      180   37  PF6
    S10           36  PF7
    S11           32  PC7
    S12           31  PC6
    S13     270   30  PB6
    S14           29  PB5
    S15           28  PB4
    S16           27  PD7
    loop cycle duration: 3.2 millis
**/

#define S5 ((PINE & 64 ) >> 6)

#define S11 ((PINC & 128 ) >> 7)
#define S12 ((PINC & 64 ) >> 6)

#define S13 ((PINB & 64 ) >> 6)
#define S14 ((PINB & 32 ) >> 5)
#define S15 ((PINB & 16) >> 4)

#define S1 ((PIND & 64 ) >> 6)
#define S2 ((PIND & 16 ) >> 4)
#define S3 ((PIND & 2 ) >> 1)
#define S4 (PIND & 1 )
#define S16 ((PIND & 128) >> 7)

#define S6 ((PINF & 2 ) >> 1)
#define S7 ((PINF & 16 ) >> 4)
#define S8 ((PINF & 32 ) >> 5)
#define S9 ((PINF & 64 ) >> 6)
#define S10 ((PINF & 128 ) >> 7)

#define NCYCLES 250
#define BROKEN  230
#define TOO_LOW 45

#define LED1ON (PORTD = PORTD | 0b00100000)
#define LED1OFF (PORTD = PORTD & 0b11011111)

#define LED2ON (PORTF = PORTF | 0b00000001)
#define LED2OFF (PORTF = PORTF & 0b11111110)

#define LED3ON (PORTB = PORTB | 0b10000000)
#define LED3OFF (PORTB = PORTB & 0b01111111)

#define LED4ON (PORTB = PORTB | 0b00000001)
#define LED4OFF (PORTB = PORTB & 0b11111110)


int counter[16];
int distance;
int nmax = 0;
int sensor = 0;

int oldIndex = 0;
int oldDistance = 0;

byte ballInfo = 0;

float xs[16];
float ys[16];

float angle = 0;
int dist = 0;
boolean sending = false;
byte sendAngle = 0, sendDistance = 0;
byte sendByte = 0;

unsigned long t = 0;

void setup() {
  delay(1000);

  Serial1.begin(57600);

  
  /*For now replace pinMode with writes to the direction register. 
    We don't know if pinMode will work on those sensors, and it has proven not be working on digitalWrite for reasons probably relative to compatibility between Arduino and our board,
    but this needs further investigation*/  

  //Set the LEDs as outputs, keep the rest as input by default

  //LED3(PB7) and LED4 (PB0)
  DDRB |= 0b10000001;
  //LED2 (PF0)
  DDRF |= 0b00000001;
  //LED1 (PD5)
  DDRD |= 0b00100000;
  
  /*pinMode(26, INPUT);   //S1
  pinMode(25, INPUT);   //S2
  pinMode(19, INPUT);   //S3
  pinMode(18, INPUT);   //S4
  pinMode(1, INPUT);    //S5
  pinMode(40, INPUT);   //S6
  pinMode(39, INPUT);   //S7
  pinMode(38, INPUT);   //S8
  pinMode(37, INPUT);   //S9
  pinMode(36, INPUT);   //S10
  pinMode(32, INPUT);   //S11
  pinMode(31, INPUT);   //S12
  pinMode(30, INPUT);   //S13
  pinMode(29, INPUT);   //S14
  pinMode(28, INPUT);   //S15
  pinMode(27, INPUT);   //S16*/
  
  for (int i = 0; i < 16; i++) {
    xs[i] = cos((22.5 * PI / 180) * i);
    ys[i] = sin((22.5 * PI / 180) * i);
  }
}

void loop() {
  readBallInterpolation();
  //printCounter();
  sendDataInterpolation();
  readFromTeensy();
  //test();
  //delay(100);
}

/**--- READ BALL USING SENSORS ANGLE INTERPOLATION ---**/

void readBallInterpolation() {
  for (int i = 0; i < 16; i++) {
    counter[i] = 0;
  }

  //reads from the register
  for (int i = 0; i < NCYCLES; i++) {
    counter[0] += !S1;
    counter[1] += !S2;
    counter[2] += !S3;
    counter[3] += !S4;
    counter[4] += !S5;
    counter[5] += !S6;
    counter[6] += !S7;
    counter[7] += !S8;
    counter[8] += !S9;
    counter[9] += !S10;
    counter[10] += !S11;
    counter[11] += !S12;
    counter[12] += !S13;
    counter[13] += !S14;
    counter[14] += !S15;
    counter[15] += !S16;
  }

  float x = 0, y = 0;
  for (int i = 0; i < 16; i++) {
    if (counter[i] > BROKEN || counter[i] < TOO_LOW) counter[i] = 0;

    x += xs[i] * counter[i];
    y += ys[i] * counter[i];
  }

  angle = atan2(y, x) * 180 / PI;
  angle = ((int)(angle + 360)) % 360;

  //distance is 0 when not seeing ball
  //dist = hypot(x, y);
  
  nmax = 0;
  //saves max value and sensor
  for (int i = 0; i < 16; i++) {
    if (counter[i] > nmax) {
      nmax = counter[i];
      sensor = i;
    }
  }

  distance = nmax;
  
  //turn led on
  if (distance == 0) {
    LED1OFF;
  } else {
    LED1ON;
  }
}

void sendDataInterpolation() {
  if(sending){
    sendAngle = ((byte) (angle / 2)) & 0b11111110;
    Serial1.write(sendAngle);
  }else{
    sendDistance = map(distance, 0, NCYCLES, 254, 0);
    sendDistance = sendDistance |= 0b00000001;
    Serial1.write(sendDistance);
  }
  sending = !sending;
}

void test() {
  readBallInterpolation();
  
  Serial1.println("===========");
  Serial1.print(S1);
  Serial1.print(" | ");
  Serial1.print(S2);
  Serial1.print(" | ");
  Serial1.print(S3);
  Serial1.print(" | ");
  Serial1.print(S4);
  Serial1.print(" | ");
  Serial1.print(S5);
  Serial1.print(" | ");
  Serial1.print(S6);
  Serial1.print(" | ");
  Serial1.print(S7);
  Serial1.print(" | ");
  Serial1.print(S8);
  Serial1.print(" | ");
  Serial1.print(S9);
  Serial1.print(" | ");
  Serial1.print(S10);
  Serial1.print(" | ");
  Serial1.print(S11);
  Serial1.print(" | ");
  Serial1.print(S12);
  Serial1.print(" | ");
  Serial1.print(S13);
  Serial1.print(" | ");
  Serial1.print(S14);
  Serial1.print(" | ");
  Serial1.print(S15);
  Serial1.print(" | ");
  Serial1.print(S16);
  Serial1.print(" ---  ");
  Serial1.println(sensor);
  Serial1.println("===========");
  delay(100);

}


void printCounter() {
  for (int i = 0; i < 16; i++) {
    Serial1.print(counter[i]);
    Serial1.print(" | ");
  }
    Serial1.print("\t\t| Angle: " );
    Serial1.print(angle);
    Serial1.print("||| Distance: " );
    Serial1.print(distance);
    
  Serial1.println();
  delay(100);
}

void readFromTeensy(){
  while(Serial1.available()){
    byte b = Serial1.read();
    if(b & 0b00000001 == 1) LED2ON;
    else LED2OFF;
    if((b & 0b00000010) >> 1 == 1) LED3ON;
    else LED3OFF;
    if((b & 0b00000100) >> 2 == 1) LED4ON;
    else LED4OFF;
  }
}
