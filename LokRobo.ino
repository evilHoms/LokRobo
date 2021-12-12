#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include "QuatroPortA100.h"
#include "RF24Service.h"

#define FORWARD 1
#define HOLD 0
#define BACKWARD -1

#define LEFT 1
#define STRAIGHT 0
#define RIGHT -1

#define ENABLE_MOTORS
//#define TEST_MOTORS
//#define ENABLE_DISTANCE_SENSOR
#define ENABLE_RF
//#define SCAN_ENABLED

#define DEFAULT_CHANNEL 0x70

#define MID_DISTANCE 200
#define CLOSE_DISTANCE 100
//#define IS_DEBUG

#define KEY 123

const int CE_PIN = 10;
const int CSE_PIN = 9;
const int M1_IN = 5;
const int M1_DIR = 7;
const int M2_IN = 6;
const int M2_DIR = 8;

byte transmitterChannel = DEFAULT_CHANNEL;
byte address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};

const byte RF_PACKET_SIZE = 8;

VL53L0X distanceSensor;
RF24Service radio(CE_PIN, CSE_PIN);
QuatroPortA100 motors(M1_IN, M1_DIR, M2_IN, M2_DIR, true, false);

typedef struct {
  bool btn;
  byte x;
  byte y;
}
Stick;

typedef struct {
  byte key;
  bool btn1;
  bool btn2;
  bool btn3;
  bool btn4;
  byte pot1;
  Stick stick1;
  Stick stick2;
}
Data;

Data data;

int mX = 0;
int mY = 0;

int m1Power = 210;
int m2Power = 255;

int moveSpeed = 100;

int scanChannels();

void initRF();
void runRF();
void parceByteData(byte byteArray[RF_PACKET_SIZE]);
void dataToSignal();

int distance = 0;

void setup() {
  Serial.begin(9600);

  #ifdef IS_DEBUG
    Serial.println("Starting...");
  #endif

  #ifdef ENABLE_DISTANCE_SENSOR
    Wire.begin();
    if (!distanceSensor.init()) {
      #ifdef IS_DEBUG
        Serial.println("Failed to init destance sensor");
      #endif
    } else {
      distanceSensor.setTimeout(500);
      #ifdef IS_DEBUG
        Serial.println("Distance sensor enabled");
      #endif
    }
  #endif

  #ifdef ENABLE_MOTORS
    pinMode(M1_IN, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_IN, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    #ifdef IS_DEBUG
      Serial.println("Motors enabled");
    #endif
  #endif
  
  #ifdef ENABLE_RF
    #ifdef IS_DEBUG
      Serial.println("Radio module enabled");
//      radio.showDebug();
    #endif

    initRF();
  #endif
}

void loop() {
  
  #ifdef ENABLE_RF
    runRF(transmitterChannel);
  #endif

  #ifdef ENABLE_DISTANCE_SENSOR
    distance = distanceSensor.readRangeSingleMillimeters();
  #endif

  #ifdef ENABLE_MOTORS
    move();
  #endif

  #ifdef IS_DEBUG
    Serial.println("/-------------------/");

    #ifdef ENABLE_MOTORS
      Serial.print("Motors: ");
      Serial.print(mX < 0 ? "RIGHT " : mY > 0 ? "LEFT " : "STRAIGHT ");
      Serial.println(mY < 0 ? "FORWARD" : mY > 0 ? "BACKWARD" : "HOLD");
    #endif

    #ifdef ENABLE_RF
      Serial.print("Data: ");
      Serial.println(0);
    #endif
    
    #ifdef ENABLE_DISTANCE_SENSOR
      Serial.print("Distance: ");
      Serial.println(distance);
    #endif

    Serial.println("/-------------------/");
  #endif

  delay(10);
}

void move() {  
  short moveDirection = HOLD;
  short turnDirection = STRAIGHT;

  #ifdef ENABLE_DISTANCE_SENSOR
    if (distance > 0 && distance < MID_DISTANCE) {
      if (distance < CLOSE_DISTANCE) {
        moveDirection = BACKWARD;
      } else {
        moveDirection = HOLD; 
      }
    }
  #endif

  #ifdef TEST_MOTORS
    motors.test();
  #endif

  #ifndef TEST_MOTORS
    dataToSignal();
  
    int m1ResultPower = m1Power * abs(mY) / 100 * moveSpeed / 100;
    int m2ResultPower = m2Power * abs(mY) / 100 * moveSpeed / 100;
    int m1TurnPower = m1ResultPower * abs(mX) / 100 * moveSpeed / 100;
    int m2TurnPower = m2ResultPower * abs(mX) / 100 * moveSpeed / 100;

    if (mY < 0) {
      moveDirection = FORWARD;
    } else if (mY > 0) {
      moveDirection = BACKWARD;
    } else {
      moveDirection = HOLD;
    }

    if (mX < 0) {
      turnDirection = RIGHT;
    } else if (mX > 0) {
      turnDirection = LEFT;
    } else {
      turnDirection = STRAIGHT;
    }

    switch (moveDirection) {
      case FORWARD: {
        if (turnDirection == LEFT) {
          motors.moveForward(m1ResultPower, m2ResultPower - m2TurnPower);
        } else if (turnDirection == RIGHT) {
          motors.moveForward(m1ResultPower - m1TurnPower, m2ResultPower);
        } else {
          motors.moveForward(m1ResultPower, m2ResultPower);
        }

        break;
      }
      case BACKWARD: {
        if (turnDirection == LEFT) {
          motors.moveBackward(m1ResultPower, m2ResultPower - m2TurnPower);
        } else if (turnDirection == RIGHT) {
          motors.moveBackward(m1ResultPower - m1TurnPower, m2ResultPower);
        } else {
          motors.moveBackward(m1ResultPower, m2ResultPower);
        }

        break;
      }
      case HOLD:
      default:
        motors.hold();
    }
  #endif
}

void initRF() {
  radio.init();
  radio.asReciever();
  radio.withPayload();

  #ifdef SCAN_ENABLED
    transmitterChannel = scanChannels();
  #endif

  Serial.println();
  Serial.print("Channel: ");
  Serial.println(transmitterChannel, HEX);

  radio.setChannel(transmitterChannel);
  radio.startListening();
}

void runRF(int channel){
  byte pipeNo;
  byte b[RF_PACKET_SIZE];
  
  while (radio.available(&pipeNo)) {
    radio.read(&b, RF_PACKET_SIZE);

    parceByteData(b);

    if (data.key == KEY) {
      byte resKey = b[0]; // Get key from request
      radio.writeAckPayload(pipeNo, &resKey, 1 );
    }
 }
}

int scanChannels () {
  static bool isSetUp = false;
  const byte numChannels = 126;
  byte values[numChannels] = {0};
  byte resultValues[numChannels] = {0};
  unsigned short scanRepeats = 2;
  int resultChannel = -1;
  byte b[RF_PACKET_SIZE];

  byte key = KEY;

  #ifdef IS_DEBUG
    if (!isSetUp) {
       Serial.println("Start Scanning for Transmitter...");
      
      // Print out header, high then low digit
      for (int i = 0; i < numChannels; i++) {
        Serial.print(i>>4);
      }
      Serial.println();
      for (int i = 0; i < numChannels; i++) {
        Serial.print(i&0xf, HEX);
      }
      Serial.println();
  
      isSetUp = true;
    }
  #endif
  
  for (int i = 0; i < scanRepeats; i ++) {
    Serial.println("");
    for (int j = 0; j < numChannels; j ++) {
      radio.setChannel(j);
      radio.startListening();

      Serial.print(".");

      byte pipeNo;
      short gotKey;
      while (radio.available(&pipeNo)) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        radio.read(&b, RF_PACKET_SIZE);

        if (b[0] == KEY) {
          byte resKey = b[0];
  
          int resultChannel = j - 1; // By some reason data we get from next channel;
          radio.writeAckPayload(pipeNo, &resKey, 1);
          Serial.println("");
          Serial.print("FOUND: ");
          Serial.println(resultChannel, HEX);
          radio.stopListening();
          return resultChannel;
        }
      }
      delay(20);
      radio.stopListening();
    }
  }
  
  Serial.println();
  
  return resultChannel;
}

void parceByteData(byte b[RF_PACKET_SIZE]) {
  if (b[7] != 0) return;
  
  data.key = b[0];
  
  byte btns[] = {1, 2, 4, 8, 16, 32};
  byte btnValue = b[1];

  data.stick2.btn = false;
  data.stick1.btn = false;
  data.btn4 = false;
  data.btn3 = false;
  data.btn2 = false;
  data.btn1 = false;

  if (btnValue >= btns[5]) {
    btnValue -= btns[5];
    data.stick2.btn = true;
  }
  if (btnValue >= btns[4]) {
    btnValue -= btns[4];
    data.stick1.btn = true;
  }
  if (btnValue >= btns[3]) {
    btnValue -= btns[3];
    data.btn4 = true;
  }
  if (btnValue >= btns[2]) {
    btnValue -= btns[2];
    data.btn3 = true;
  }
  if (btnValue >= btns[1]) {
    btnValue -= btns[1];
    data.btn2 = true;
  }
  if (btnValue >= btns[0]) {
    btnValue -= btns[0];
    data.btn1 = true;
  }

  data.pot1 = b[2];
  data.stick1.x = b[3];
  data.stick1.y = b[4];
  data.stick2.x = b[5];
  data.stick2.y = b[6];
}

void dataToSignal() {
  mY = (int(float(data.stick1.y) / 255.0 * 200.0) / 10 * 10 - 100) * -1; // from 0 - 255 to -100 - 100; reverse; round to ten
  mX = (int(float(data.stick1.x) / 255.0 * 200.0) / 10 * 10 - 100) * -1; // -100 - 100;
  moveSpeed = 100 - int(float(data.pot1) / 255.0 * 100.0); // 0 - 100;
}
