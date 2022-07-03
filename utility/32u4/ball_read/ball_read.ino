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
#define S10 ((PINB & 1))
#define S9 ((PINB & 2) >> 1)
#define S8 ((PINB & 4) >> 2)
#define S7 ((PINB & 8) >> 3)
#define S6 ((PINB & 16) >> 4)
#define S5 ((PINB & 32) >> 5)

#define S4 ((PINC & 1))
#define S3 ((PINC & 2) >> 1)
#define S2 ((PINC & 4) >> 2)
#define S1 ((PINC & 8) >> 3)

#define S16 ((PIND & 4) >> 2)
#define S15 ((PIND & 8) >> 3)
#define S14 ((PIND & 16) >> 4)
#define S13 ((PIND & 32) >> 5)
#define S12 ((PIND & 64) >> 6)
#define S11 ((PIND & 128) >> 7)

#define NCYCLES 255
#define BROKEN  230
#define TOO_LOW 45

byte counter[16];
byte distance;
byte nmax = 0;
int sensor = 0;

int oldIndex = 0;
int oldDistance = 0;

float xs[16];
float ys[16];

float angle = 0;
byte dist = 0;

unsigned long t = 0;
byte buf[3];

void setup() {
  delay(1000);

  Serial.begin(57600);

  
  /*For now replace pinMode with writes to the direction register. 
    We don't know if pinMode will work on those sensors, and it has proven not be working on digitalWrite for reasons probably relative to compatibility between Arduino and our board,
    but this needs further investigation*/  

  //Set the LEDs as outputs, keep the rest as input by default
  pinMode(A4, OUTPUT);
  
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
  angle = ( ((int)(angle) + 360)) % 360;

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

  distance = NCYCLES - nmax;
  
  //turn led on
  if (distance == NCYCLES) {
    digitalWrite(A4, LOW);
  } else {    
    digitalWrite(A4, HIGH);
  }
}

void sendDataInterpolation() {
  buf[0] = 0b01000000;
  buf[1] = 0b10000000;
  buf[2] = 0b11000000;

  buf[0] |= ((int)angle) & 0b000111111;
  buf[1] |= (((int)angle) & 0b111000000) >> 6;
  buf[1] |= (distance & 0b11100000) >> 2;
  buf[2] |= distance & 0b00011111;
  
  if(Serial.availableForWrite() >= 3) Serial.write(buf, 3);
}

void test() {
  readBallInterpolation();
  
  Serial.println("===========");
  Serial.print(S1);
  Serial.print(" | ");
  Serial.print(S2);
  Serial.print(" | ");
  Serial.print(S3);
  Serial.print(" | ");
  Serial.print(S4);
  Serial.print(" | ");
  Serial.print(S5);
  Serial.print(" | ");
  Serial.print(S6);
  Serial.print(" | ");
  Serial.print(S7);
  Serial.print(" | ");
  Serial.print(S8);
  Serial.print(" | ");
  Serial.print(S9);
  Serial.print(" | ");
  Serial.print(S10);
  Serial.print(" | ");
  Serial.print(S11);
  Serial.print(" | ");
  Serial.print(S12);
  Serial.print(" | ");
  Serial.print(S13);
  Serial.print(" | ");
  Serial.print(S14);
  Serial.print(" | ");
  Serial.print(S15);
  Serial.print(" | ");
  Serial.print(S16);
  Serial.print(" ---  ");
  Serial.println(sensor);
  Serial.println("===========");
  delay(100);

}


void printCounter() {
  for (int i = 0; i < 16; i++) {
    Serial.print(counter[i]);
    Serial.print(" | ");
  }
    Serial.print("\t\t| Angle: " );
    Serial.print(angle);
    Serial.print("||| Distance: " );
    Serial.print(distance);
    
  Serial.println();
  delay(100);
}
