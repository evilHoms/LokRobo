#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h" 

#define FORWARD HIGH
#define BACKWARD LOW

#define ENABLE_MOTORS
#define ENABLE_DISTANCE_SENSOR
//#define ENABLE_BT
#define ENABLE_RF

#define SCAN_ENABLED
#define DEFAULT_CHANNEL 0x60

#define MID_DISTANCE 200
#define CLOSE_DISTANCE 100

#define KEY 1234

const int CE_PIN = 10;
const int CSE_PIN = 9;
const int M1_IN = 5;
const int M1_DIR = 7;
const int M2_IN = 6;
const int M2_DIR = 8;

VL53L0X distanceSensor;
RF24 radio(CE_PIN, CSE_PIN);

typedef struct {
  int key;
  bool btn1;
  bool btn2;
  bool btn3;
}
Data;

Data data;

byte address[][6] = { "1Node", "2Node", "3Node", "4Node", "5Node", "6Node" };

int mX = 0;
int mY = 0;

int m1Power = 210;
int m2Power = 255;

int moveSpeed = 100;

int scanChannels();
void parceCommand(String command);
void moveForward(int m1Power, int m2Power);
void moveBackward(int m1Power, int m2Power);

void initRF();
void runRF();
void parceRfResponse(Data data);

int distance = 0;

String command = "";
bool isReadingCommand = false;
char line[255];
int lineIndex = 0;

void setup() {
  Serial.begin(9600);

  #ifdef ENABLE_DISTANCE_SENSOR
    Wire.begin();
    distanceSensor.init();
    distanceSensor.setTimeout(500);
  #endif

  #ifdef ENABLE_MOTORS
    pinMode(M1_IN, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_IN, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
  #endif
  
  #ifdef ENABLE_RF
    initRF();
  #endif
}

void loop() {
  
  #ifdef ENABLE_RF
    runRF();
  #endif

  #ifdef ENABLE_BT
    runBT();
  #endif

  #ifdef ENABLE_DISTANCE_SENSOR
    distance = distanceSensor.readRangeSingleMillimeters();
  #endif

  #ifdef ENABLE_MOTORS
    move();
  #endif

  delay(10);
}

void moveForward(int m1Power, int m2Power) {
  analogWrite(M1_IN, 255 - m1Power);
  digitalWrite(M1_DIR, FORWARD);
  analogWrite(M2_IN, 255 - m2Power);
  digitalWrite(M2_DIR, FORWARD);
}

void moveBackward(int m1Power, int m2Power) {
  analogWrite(M1_IN, m1Power);
  digitalWrite(M1_DIR, BACKWARD);
  analogWrite(M2_IN, m2Power);
  digitalWrite(M2_DIR, BACKWARD);
}

void hold() {
  analogWrite(M1_IN, 0);
  digitalWrite(M1_DIR, BACKWARD);
  analogWrite(M2_IN, 0);
  digitalWrite(M2_DIR, BACKWARD);
}

void move() {  
  bool isHold = false;
  bool isBackward = false;

  #ifdef ENABLE_DISTANCE_SENSOR
    if (distance > 0 && distance < MID_DISTANCE) {
      if (distance < CLOSE_DISTANCE) {
        isBackward = true;
      } else {
        isHold = true; 
      }
    }
  #endif
  
  int m1ResultPower = m1Power * abs(mY) / 100 * moveSpeed / 100;
  int m2ResultPower = m2Power * abs(mY) / 100 * moveSpeed / 100;
  int m1TurnPower = m1ResultPower * abs(mX) / 100 * moveSpeed / 100;
  int m2TurnPower = m2ResultPower * abs(mX) / 100 * moveSpeed / 100;
  bool isForward = mY < 0;
  
  if (isForward && !isHold && !isBackward) {
    if (mX > 0) {
      moveForward(m1ResultPower, m2ResultPower - m2TurnPower);
    } else if (mX < 0) {
      moveForward(m1ResultPower - m1TurnPower, m2ResultPower);
    } else {
      moveForward(m1ResultPower, m2ResultPower);
    }
  } else if (!isForward || isBackward) {
    if (mX > 0) {
      moveBackward(m1ResultPower, m2ResultPower - m2TurnPower);
    } else if (mX < 0) {
      moveBackward(m1ResultPower - m1TurnPower, m2ResultPower); 
    } else {
      moveBackward(m1ResultPower, m2ResultPower);
    }
  } else if (isHold) {
    hold();
  }
}

void parceCommand(String command) {
  command.remove(command.length() - 2, 2);
  switch (command.charAt(0)) {
    case 'j': {
      mX = map(constrain(command.substring(2, 5).toInt(), 0, 255), 0, 255, -100, 100);
      mY = map(constrain(command.substring(5, 9).toInt(), 0, 255), 0, 255, -100, 100);
      break;
    }
    case 'k': {
      switch (command.charAt(2)) {
        case '1': {
          mX = -50;
          mY = -100;
          break;
        }
        case '2': {
          mX = 0;
          mY = -100;
          break;
        }
        case '3': {
          mX = 50;
          mY = -100;
          break;
        }
        case '4': {
          mX = -100;
          mY = -80;
          break;
        }
        case '5': {
          mX = 0;
          mY = 0;
          break;
        }
        case '6': {
          mX = 100;
          mY = -80;
          break;
        }
        case '7': {
          mX = -50;
          mY = 100;
          break;
        }
        case '8': {
          mX = 0;Serial.println(data.btn1);
//    Serial.println(data.btn2);
//    Serial.println(data.btn3);
//    Serial.println(data.key);
          mY = 100;
          break;
        }
        case '9': {
          mX = 50;
          mY = 100;
          break;
        }
        default: {
          mX = 0;
          mY = 0;
        }
      }
      break;
    }
    case 'r': {
      if (command.charAt(1) == '0') {
        moveSpeed = map(constrain(command.substring(2, 5).toInt(), 0, 255), 0, 255, 0, 100);
        Serial.print("Speed: ");
        Serial.println(moveSpeed);
      }
      break;
    }
    default: {
      Serial.println("Unknown command");  
    }
  }
}

void runBT() {
  if (Serial.available()) {
    if (!isReadingCommand) {
      command = "";
      isReadingCommand = true;
    }
    line[lineIndex] = (char)Serial.read();
    lineIndex += 1;

    // If end of line
    if (lineIndex > 1 && line[lineIndex - 1] == 10 && line[lineIndex - 2] == 13) {
      parceCommand(String(line));
      isReadingCommand = false;
      lineIndex = 0;
      line[0] = '\0';
    }
  }
}

void initRF() {
  Serial.println("Init RF");
  radio.begin(); //активировать модуль
  radio.setAutoAck(1);         //режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0,3);     //(время между попыткой достучаться, число попыток)
  radio.enableAckPayload();    //разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);     //размер пакета, в байтах

  radio.openReadingPipe(1,address[0]);      //хотим слушать трубу 0

  radio.setPALevel (RF24_PA_MAX); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_1MBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  // ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!
  
  radio.powerUp(); //начать работу
}

void runRF(){
  static bool isScanning = true;

  if (isScanning) {
    int transmitterChannel = DEFAULT_CHANNEL;
    
    #ifdef SCAN_ENABLED
     transmitterChannel = scanChannels();
    #endif
    
    if (transmitterChannel != -1) {
      Serial.println();
      Serial.print("Connected to transmitter: ");
      Serial.println(transmitterChannel, HEX);
      radio.setChannel(transmitterChannel);  // Устанавливаем канал
      radio.startListening();  //начинаем слушать эфир, мы приёмный модуль
      isScanning = false;
    } else {
      Serial.println("Transmitter wasn't found!");
      delay(1000);
      Serial.println("Retry...");
    }

    //isScanning = false; // Удалить, когда заработает
  }
  
  byte pipeNo;
  while (radio.available(&pipeNo)) {    // слушаем эфир со всех труб
    radio.read( &data, sizeof(data) );         // чиатем входящий сигнал
    bool isCorrectKey = data.key == KEY ? true : false;

    radio.writeAckPayload(pipeNo, &isCorrectKey, 1 );  // ответ на запрос
    Serial.println("Recieved: ");
    Serial.println(data.btn1);
    Serial.println(data.btn2);
    Serial.println(data.btn3);
    Serial.println(data.key);

    parceRfResponse(data);
 }
}

void parceRfResponse(Data data) {
  if (data.btn1) {
    Serial.println("Left");
    mX = -100;
    mY = -80;
  } else if (data.btn2) {
    Serial.println("Forward");
    mX = 0;
    mY = -100;
  } else if (data.btn3) {
    Serial.println("Right");
    mX = 100;
    mY = -80;
  }

  if (!data.btn1 && !data.btn2 && !data.btn3) {
    mX = 0;
    mY = 0;
  }
}

int scanChannels () {
  static bool isSetUp = false;
  const byte numChannels = 126;
  byte values[numChannels] = {0};
  byte resultValues[numChannels] = {0};
  unsigned short scanRepeats = 10;
  int resultChannel = -1;

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
  
  for (int i = 0; i < scanRepeats; i ++) {
    for (int j = 0; j < numChannels; j ++) {
      radio.setChannel(j);
      radio.startListening();
      delay(10);
      byte pipeNo;
      while (radio.available(&pipeNo)) {
        Serial.println("Available");
        Serial.println(j);
        radio.read( &data, sizeof(data) );         // Listen for signal
        bool isCorrectKey = data.key == KEY ? true : false;  // If keys equal, it is our channel
        Serial.println("FOUND!");
        Serial.println(j, HEX);
        resultChannel = j;
        i = scanRepeats;
        j = numChannels;
      }
      radio.stopListening();
    }
  }
  
  Serial.println();
  
  return resultChannel;
}
