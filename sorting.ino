#include <Wire.h>
#include "Adafruit_TCS34725.h"  //color sensor

#include <MX1508.h>
uint8_t searchAddress = 0x29;   //  i2c address
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
const int colorIntPin = A3;
volatile boolean colorint = true;
volatile boolean topint = false;
volatile boolean botint = false;
volatile uint8_t prevValue = 0;


#define NUM_MOTORS 3
#define B_PINA 5
#define B_PINB 3
#define C_PINA 6
#define C_PINB 9
#define A_PINA 11
#define A_PINB 10
MX1508 motor[NUM_MOTORS] = {
  MX1508(A_PINA, A_PINB),
  MX1508(B_PINA, B_PINB),
  MX1508(C_PINA, C_PINB)
};


#define UINT32_MAX 0xFFFFFFFF

#define DEBUG_SKITTLES 1

#if DEBUG_SKITTLES
#define DUMP(s, v) \
  { \
    Serial.print(F(s)); \
    Serial.print(v); \
  }
#define DUMPBIN(s, v) \
  { \
    Serial.print(F(s)); \
    Serial.print(v,BIN); \
  }  
#define DUMPDEC(s, v) \
  { \
    Serial.print(F(s)); \
    Serial.print(v,DEC); \
  }    
#define DUMPS(s) Serial.print(F(s))
#define DUMPSLN(s) Serial.println(F(s))
#else
#define DUMP(s, v)
#define DUMPS(s)
#endif


#define TOPDISK 0
#define BOTDISK 1
#define HOLE false
#define SPACE true
#define FORWARD 1
#define BACKWARD 0

#define BLACK_CAL 0
#define WHITE_CAL 1
#define READ_VAL 2

#define NUM_COLORS 5
#define NUM_RGB 3
uint8_t skittles[NUM_COLORS][NUM_RGB] = {
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

float scale_factor[NUM_RGB] = { 0.0, 0.0, 0.0 };

// Interrupt definitions, D2 and D4
#define TOPDISK_INT 4
#define BOTDISK_INT 2

// Pin definitions
#define COLOR_SENSOR 5
#define TOPDISK_PIN 4
#define BOTDISK_PIN 2
#define TRIPWIRE_PIN A7 //skittle drop
#define TRIPWIRE_LEDR A0
#define TRIPWIRE_LEDG A1
#define TRIPWIRE_LEDB A2

#define RED_BUTTON 7
#define GREEN_BUTTON 8

volatile uint8_t diskDir[2] = { FORWARD, FORWARD };
volatile uint8_t diskPos[2] = { 0, 0 };
volatile uint8_t targetPos[2] = { 0, 0 };

bool isStop = false;
bool isInitialized = false;

/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
  DUMPDEC("crgb:",*c);
  DUMPDEC(",",*r);
  DUMPDEC(",",*g);
  DUMPDEC(",",*b);
  DUMPSLN("");
}

void getRGBNormalized(float *r, float *g, float *b)
{
  uint16_t red, green, blue, clear;
  getRawData_noDelay(&red, &green, &blue, &clear);

  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = *g = *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;

}

void set_abs_pos(uint8_t disk, uint8_t pos) {
  diskPos[disk] = pos;
}

void set_pos(uint8_t disk, uint8_t pos, bool isSpace) {
  int idx = 0;
  isSpace ? idx = 1 : idx = 0;
  diskPos[disk] = 2 * pos + idx;
}

uint8_t get_abs_pos(uint8_t disk) {
  return diskPos[disk];
}

void add_abs_pos(uint8_t disk, int num) {
  diskPos[disk] = mod(diskPos[disk] + num, 10);
}

uint8_t get_pos(uint8_t disk) {
  return diskPos[disk] / 2;
}

bool is_space(uint8_t disk) {
  return ((diskPos[disk] % 2) == 1);
}

void set_abs_target_pos(uint8_t disk, uint8_t pos) {
  targetPos[disk] = pos;
}

void set_target_pos(uint8_t disk, uint8_t pos, bool isSpace) {
  int idx = 0;
  isSpace ? idx = 1 : idx = 0;  
  targetPos[disk] = 2 * pos + idx;
}

uint8_t get_abs_target_pos(uint8_t disk) {
  return targetPos[disk];
}

void add_abs_target_pos(uint8_t disk, int num) {
  targetPos[disk] = mod(targetPos[disk] + num, 10);
}

uint8_t get_target_pos(uint8_t disk) {
  return targetPos[disk] / 2;
}

bool is_space_target(uint8_t disk) {
  return ((targetPos[disk] % 2) == 1);
}

bool is_target(uint8_t disk) {
  return (targetPos[disk] == diskPos[disk]);
}

void top_pos_handler() {
  //DUMPSLN("top_pos_handler");
  int value = digitalRead(TOPDISK_PIN);
  if (!value) {
    if ((diskDir[TOPDISK] == BACKWARD) && is_space(TOPDISK)) {
      add_abs_pos(TOPDISK, -1);
    } else if ((diskDir[TOPDISK] == FORWARD) && !is_space(TOPDISK)) {
      add_abs_pos(TOPDISK, 1);
    }
  } else {
    if ((diskDir[TOPDISK] == BACKWARD) && !is_space(TOPDISK)) {
      add_abs_pos(TOPDISK, -1);
    } else if ((diskDir[TOPDISK] == FORWARD) && is_space(TOPDISK)) {
      add_abs_pos(TOPDISK, 1);
    }
  }
  DUMP("TopPos:", get_abs_pos(TOPDISK));
  DUMPSLN("");  
}

void bot_pos_handler() {
  //DUMPSLN("bot_pos_handler");
  int value = digitalRead(BOTDISK_PIN);
  if (!value) {
    if ((diskDir[BOTDISK] == BACKWARD) && is_space(BOTDISK)) {
      add_abs_pos(BOTDISK, -1);
    } else if ((diskDir[BOTDISK] == FORWARD) && !is_space(BOTDISK)) {
      add_abs_pos(BOTDISK, 1);
    }
  } else {
    if ((diskDir[BOTDISK] == BACKWARD) && !is_space(BOTDISK)) {
      add_abs_pos(BOTDISK, -1);
    } else if ((diskDir[BOTDISK] == FORWARD) && is_space(BOTDISK)) {
      add_abs_pos(BOTDISK, 1);
    }
  }
  DUMP("BotPos:", get_abs_pos(BOTDISK));
  DUMPSLN("");
}


void setup() {
  pinMode(RED_BUTTON, INPUT);        // set pin to input
  digitalWrite(RED_BUTTON, HIGH);    // turn on pullup resistors
  pinMode(GREEN_BUTTON, INPUT);      // set pin to input
  digitalWrite(GREEN_BUTTON, HIGH);  // turn on pullup resistors

  //Replaced with PCINTs due to limited PWM on the Nano consumes D3 (D2 and D3 are the only HW interrupts)
  //attachInterrupt(TOPDISK_INT, top_pos_handler, CHANGE);
  //attachInterrupt(BOTDISK_INT, bot_pos_handler, CHANGE);

  pinMode(colorIntPin, INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain
  pinMode(TOPDISK_INT, INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain
  pinMode(BOTDISK_INT, INPUT_PULLUP); //TCS interrupt output is Active-LOW and Open-Drain

  // https://www.electrosoftcloud.com/en/pcint-interrupts-on-arduino/
  PCICR |= 0b00000110; // We activate the interrupts of the PC and PD
  PCMSK1 |= 0b00001000; // Trigger interrupts on pins A3
  PCMSK2 |= 0b00010100; // Trigger interrupts on pins D2 and D4 for disks

  Serial.begin(38400);
  DUMPSLN("Start skittles sorter");

  Wire.begin();

  if (tcs.begin()) {
    DUMPSLN("Initializing color sensor...");
  } else {
    DUMPSLN("No TCS34725 found... check your connections");
    while (1);
  }  
  DUMPSLN("i2c success!");

  //finish initilizing the color sensor
  // Set persistence filter to generate an interrupt for every RGB Cycle, regardless of the integration limits
  tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE); 
  tcs.setInterrupt(true);
  sei();

  delay(100);

  tcs.clearInterrupt();
  colorint = false;

}
void tripOff(){
  //turn off tripwire led
  analogWrite(TRIPWIRE_LEDR, 0);
  analogWrite(TRIPWIRE_LEDG, 0);
  analogWrite(TRIPWIRE_LEDB, 0);    
}
void tripOn(){
  //turn on tripwire led "white"
  analogWrite(TRIPWIRE_LEDR, 200);
  analogWrite(TRIPWIRE_LEDG, 200);
  analogWrite(TRIPWIRE_LEDB, 200);
}

void loop() {
  while (isStop) {
    tripOff();
    if (digitalRead(GREEN_BUTTON) == LOW){
      isStop = false;
    }
  }

  if (!isInitialized) {
    DUMPSLN("Initializing...");
    calibrate_color();      
    calibrate_positions();
    DUMPSLN("Initialization Complete!");
  }
  drop_skittle();
  move_to_color_sensor();
  int index = readColor();

  DUMP("initial botPos is ", get_abs_pos(BOTDISK));
  DUMP("\ninitial topPos is ", get_abs_pos(TOPDISK));
  DUMPS("\n");

  go_both(index, HOLE);
  go_to(BOTDISK, index, SPACE);

  DUMP("final botPos is ", get_abs_pos(BOTDISK));
  DUMP("\nfinal topPos is ", get_abs_pos(TOPDISK));
  DUMPS("\n");
}

void drop_skittle() {
  DUMPSLN("Dropping skittle...");
  tripOn();
  if (isStop) { return; }

  if (get_abs_pos(BOTDISK) == 7 || get_abs_pos(BOTDISK) == 8) {
    // move bottom disk away from feeder and color sensor
    go_to(BOTDISK, 4, SPACE);
  }
  if (is_space(TOPDISK)) {
    // move to top disk to hole first if not already
    go(TOPDISK, FORWARD, 1, HOLE);
  }

  motor[2].motorGo(255);

  // Wait for Skittle
  unsigned long startTime = millis();
  //DUMP("TRIP:", analogRead(TRIPWIRE_PIN));
  DUMPSLN("");
  while (analogRead(TRIPWIRE_PIN) > 1200) {
    if (millis() > startTime + 5000) {
      motor[0].stopMotor();
      motor[1].stopMotor();
      motor[2].stopMotor();
      isStop = true;
      DUMPSLN("TRIPWIRE FAILED!");
      return;
    }
    if (checkStop()) return;
  }
  //while(digitalRead(TRIPWIRE_PIN) == HIGH){ }
  tripOff();
  motor[2].stopMotor();

  // Wait briefly to make sure skittle has
  // fallen in hole before moving disk
  delay(250);

  // top disk position is now at feeder
  set_abs_pos(TOPDISK, 8);
}

void move_to_color_sensor() {
  go_to(TOPDISK, 3, SPACE);
}

void go_360(int disk, uint8_t dir, bool isSpace) {
  if (isStop) { return; }

  runDisk(disk, dir, 255);
  for (int i = 0; i < 5; i++) {
    if (dir == FORWARD) {
      set_target_pos(disk, mod(get_pos(disk) + 1, 5), isSpace);
    } else {
      set_target_pos(disk, mod(get_pos(disk) - 1, 5), isSpace);
    }
    while (!is_target(disk)) {
      checkStop();
    }
  }
  motor[disk].stopMotor();
}

bool checkStop() {
  if (isStop || digitalRead(RED_BUTTON) == LOW) {
    motor[0].stopMotor();
    motor[1].stopMotor();
    motor[2].stopMotor();
    isStop = true;
    isInitialized = false;
    DUMPSLN("Stop Pressed!");
    return true;
  }
  return false;
}

void go(int disk, uint8_t dir, int num, bool isSpace) {
  if (isStop) { return; }

  DUMP("go(", disk==TOPDISK ? "TOPDISK" : "BOTDISK");
  DUMP(",", dir==FORWARD ? "FORWARD": "BACKWARD");  
  DUMP(",num=", num);
  DUMP(",isSpace=", isSpace);
  DUMPS(")\n");

  if (dir == FORWARD) {
    go_to(disk, mod(get_pos(disk) + num, 5), isSpace);
  } else {
    go_to(disk, mod(get_pos(disk) - num, 5), isSpace);
  }
}

void go_to(int disk, uint8_t pos, bool isSpace) {
  DUMP("go_to(", disk==TOPDISK ? "TOPDISK" : "BOTDISK");
  DUMP(",pos=", pos);
  DUMP(",isSpace=", isSpace);
  DUMPS(")\n");

  if (isStop) { return; }

  set_target_pos(disk, pos, isSpace);

  uint8_t dir;
  if (mod(get_abs_target_pos(disk) - get_abs_pos(disk), 10) > mod(get_abs_pos(disk) - get_abs_target_pos(disk), 10)) {
    dir = BACKWARD;
  } else {
    dir = FORWARD;
  }

  //motor[disk].setSpeed(255);
  runDisk(disk, dir, 255);

  DUMP("(currentPos,targetPos) = (", get_abs_pos(disk));
  DUMP(",", get_abs_target_pos(disk));
  DUMPS(")\n");

  while (!is_target(disk)) {
    if (checkStop()) return;
  }

  DUMPS("Reverse\n");
  long speed = 0;
  add_abs_pos(disk, (dir == FORWARD) ? 1 : -1);
  if (dir == FORWARD) {
    speed = 180;
  } else {
    speed = 180;
  }
  runDisk(disk, opposite_dir(dir), speed);

  while (!is_target(disk)) {
    if (checkStop()) return;
  }

  DUMPS("Reverse Again\n");
  speed = 0;
  add_abs_pos(disk, (dir == BACKWARD) ? 1 : -1);
  if (dir == FORWARD) {
    speed = 120;
  } else {
    speed = 120;
  }
  runDisk(disk, dir, speed);

  while (!is_target(disk)) {
    if (checkStop()) return;
  }

  if (dir == FORWARD) {
    DUMPS("Reverse Again!!!\n");

    add_abs_pos(disk, 1);
    runDisk(disk, opposite_dir(dir), 100);

    while (!is_target(disk)) {
      if (checkStop()) return;
    }
  }

  motor[disk].stopMotor();

  DUMPS("Done\n");
}

void go_both(uint8_t pos, bool isSpace) {
  if (isStop) { return; }
  DUMP("go_both(pos", pos);
  DUMP(",isSpace=",isSpace);
  DUMPSLN(")");
  set_target_pos(TOPDISK, pos, isSpace);
  set_target_pos(BOTDISK, pos, isSpace);

  uint8_t dirTop, dirBot;
  if (mod(get_abs_target_pos(TOPDISK) - get_abs_pos(TOPDISK), 10) > mod(get_abs_pos(TOPDISK) - get_abs_target_pos(TOPDISK), 10)) {
    dirTop = BACKWARD;
  } else {
    dirTop = FORWARD;
  }
  if (mod(get_abs_target_pos(BOTDISK) - get_abs_pos(BOTDISK), 10) > mod(get_abs_pos(BOTDISK) - get_abs_target_pos(BOTDISK), 10)) {
    dirBot = BACKWARD;
  } else {
    dirBot = FORWARD;
  }

  DUMP("go_both *", dirTop);
  DUMP("*", dirBot);
  DUMP("* (currPosTop,curPosBot) = (", get_abs_pos(TOPDISK));
  DUMP(",", get_abs_pos(BOTDISK));
  DUMP(") -> (targetPosTop,targetPosBot) = (", get_abs_target_pos(TOPDISK));
  DUMP(",", get_abs_target_pos(BOTDISK));
  DUMPS(")\n");


  int numTop = 0;
  int numBot = 0;
  runDisk(TOPDISK, dirTop, 255);
  runDisk(BOTDISK, dirBot, 255);
  while (numTop < 4 || numBot < 4) {
    if (checkStop()) return;

    if (numTop < 4 && is_target(TOPDISK)) {
      numTop++;
      DUMP("numTop:", numTop);
      DUMPS("\n");
      uint8_t dir;
      switch (numTop) {
        case 1:
          DUMPS("TOP Reverse\n");
          dir = opposite_dir(dirTop);
          add_abs_pos(TOPDISK, (dir == FORWARD) ? -1 : 1);
          runDisk(TOPDISK, dir, 180);
          break;
        case 2:
          DUMPS("TOP Reverse Again\n");
          dir = dirTop;
          add_abs_pos(TOPDISK, (dir == FORWARD) ? -1 : 1);
          runDisk(TOPDISK, dir, 100);
          break;
        case 3:
          if (dirTop == FORWARD) {
            DUMPS("TOP Reverse Again!!!\n");
            dir = opposite_dir(dirTop);
            add_abs_pos(TOPDISK, (dir == FORWARD) ? -1 : 1);
            runDisk(TOPDISK, dir, 100);
          } else {
            numTop++;
            DUMP("numTop:", numTop);
            DUMPS("\n");
            motor[TOPDISK].stopMotor();
            DUMPS("DoneTop\n");
          }
          break;
        default:
          motor[TOPDISK].stopMotor();
          DUMPS("DoneTop\n");
      }
    }
    if (numBot < 4 && is_target(BOTDISK)) {
      numBot++;
      DUMP("numBot:", numBot);
      DUMPS("\n");
      uint8_t dir;
      switch (numBot) {
        case 1:
          DUMPS("BOT Reverse\n");
          dir = opposite_dir(dirBot);
          add_abs_pos(BOTDISK, (dir == FORWARD) ? -1 : 1);
          runDisk(BOTDISK, dir, 180);
          break;
        case 2:
          DUMPS("BOT Reverse Again\n");
          dir = dirBot;
          add_abs_pos(BOTDISK, (dir == FORWARD) ? -1 : 1);
          runDisk(BOTDISK, dir, 100);
          break;
        case 3:
          if (dirBot == FORWARD) {
            DUMPS("BOT Reverse Again!!!\n");
            dir = opposite_dir(dirBot);
            add_abs_pos(BOTDISK, (dir == FORWARD) ? -1 : 1);
            runDisk(BOTDISK, dir, 100);
          } else {
            numBot++;
            DUMP("numBot:", numBot);
            DUMPS("\n");
            motor[BOTDISK].stopMotor();
            Serial.println("DoneBot");
          }
          break;
        default:
          motor[BOTDISK].stopMotor();
          DUMPS("DoneBot\n");
      }
    }
  }
}

void pause(){
  while(true) // remain here until told to break
  {
    if(Serial.available() > 0) // did something come in?
      if(Serial.read() == 'c') // is that something the char c?
        break;
  }
}

void calibrate_positions() {
  if (isStop) { return; }
  DUMPSLN("calibrate_positions");
  go(TOPDISK, BACKWARD, 1, SPACE);
  uint32_t brightness_min = 9999;
  int white_index = 0;
  
  for (int i = 0; i < NUM_COLORS; i++) {
    set_target_pos(BOTDISK, mod(get_pos(BOTDISK) - 1, 5), SPACE);
    go(BOTDISK, BACKWARD, 1, SPACE);
    while (!is_target(BOTDISK)) {
      if (checkStop()) return;
    }
    motor[BOTDISK].stopMotor();
    uint32_t brightness = readClearChannel();
    DUMP("Brightness is: ", brightness);
    DUMPS("\n");
    if (brightness < brightness_min) {
      brightness_min = brightness;
      white_index = i;
    }
  }
  motor[BOTDISK].stopMotor();
  go(BOTDISK, BACKWARD, mod(white_index, NUM_COLORS), SPACE);
  go(TOPDISK, BACKWARD, 1, HOLE);
  delay(100);
  if (!isStop) {
    set_abs_pos(TOPDISK, 8);
    set_abs_pos(BOTDISK, 9);
    isInitialized = true;
  }
}

uint16_t readClearChannel() {
  uint16_t red, green, blue, clear;
  //clear interrupt to get the latest color
  tcs.clearInterrupt();
  colorint = false;
  while (!colorint) { 
  };
  getRawData_noDelay(&red, &green, &blue, &clear);

  return clear;
}


void calibrate_color() {
  float fr, fg, fb;
  DUMPSLN("calibrate_color");
  if (isStop) { return; }

  // Put this on a hole to clear old skittles
  go(BOTDISK, BACKWARD, 1, HOLE);

  for (int i = 0; i < NUM_COLORS; i++) {
    go(TOPDISK, BACKWARD, 1, HOLE);
    //clear interrupt to get the latest color
    tcs.clearInterrupt();
    colorint = false;

    //wait for interrupt
    while (!colorint) {
      if (checkStop()) return;
      //DUMPSLN("waiting for color");
    };

    //get color
    getRGBNormalized(&fr,&fg,&fb);
    skittles[i][0] = int(fr);
    skittles[i][1] = int(fg);
    skittles[i][2] = int(fb);
  }
  DUMPSLN("Reference Colors:");
  for (int i = 0; i < NUM_COLORS; i++){
    DUMP("Pos",i);
    DUMPDEC(": [",skittles[i][0]);
    DUMPDEC(",",skittles[i][1]);
    DUMPDEC(",",skittles[i][2]);
    DUMPSLN("]");
  }
}

void runDisk(int disk, uint8_t dir, long speed) {
  DUMP("runDisk(", disk==TOPDISK ? "TOPDISK" : "BOTDISK");
  DUMP(",", dir==FORWARD ? "FORWARD": "BACKWARD");
  DUMP(",speed=", speed);
  DUMPS(")\n");

  diskDir[disk] = dir;
  if (dir == FORWARD){
    speed = abs(speed);
  }
  if (dir == BACKWARD){
    speed = speed * -1;
  }
  DUMP("Raw speed:", speed);
  DUMPSLN("");
  motor[disk].motorGo(speed);
}

uint8_t opposite_dir(uint8_t dir) {
  return (dir == FORWARD) ? BACKWARD : FORWARD;
}


// % is the remainder operation
// this function is for modulo
// (behaves differently for negative numbers)
long mod(long a, long b) {
  return (a % b + b) % b;
}

////////////////////
// Color Functions
////////////////////

int getColor(float r, float g, float b) {
  int lowest_distance = 9999;
  int color_num;

  DUMP("getColor(r=",r);
  DUMP(",g=", g);
  DUMP(",b=", b);
  DUMPS(")\n");

  for (int i = 0; i < NUM_COLORS; i++) {

    int d = sqrt(sq(r - skittles[i][0]) + sq(g - skittles[i][1]) + sq(b - skittles[i][2]));
    DUMP("distance for color ", i);
    DUMP(" is ", d);
    DUMPS("\n");

    if (d < lowest_distance) {
      lowest_distance = d;
      color_num = i;
    }
  }

  Serial.print("Color index is ");
  Serial.println(color_num);

  return color_num;
}

int readColor() {
  float fr,fg,fb;

  //reset interupt
  tcs.clearInterrupt();
  colorint = false;

  while (!colorint) {};
  getRGBNormalized(&fr,&fg,&fb);
  printRGB(fr,fg,fb);

  return getColor(fr,fg,fb);
}

void printRGB(uint8_t r, uint8_t g, uint8_t b) {
  DUMPS("\n");
  DUMPDEC("Raw is [", r);
  DUMPDEC(",", g);
  DUMPDEC(",", b);
  DUMPS("]\n");
}

ISR (PCINT2_vect) {
 // DUMPBIN("PCINT2 INT:", PIND);
 // DUMPSLN("");
  //detect only changes
  uint8_t currentValue = PIND ^ prevValue;
  if (currentValue & 0b00010000) {
    //topint = true; //D2
    //DUMPSLN("Top Interrupt");
    top_pos_handler();
  }
  if (currentValue & 0b00000100) {
    //botint = true; //D4
    //DUMPSLN("Bottom Interrupt");
    bot_pos_handler();
  }
  prevValue = PIND;
}

ISR (PCINT1_vect) {
  //DUMPBIN("PCINT1 INT:", PINC);
  //DUMPSLN("");  
  colorint = true;
  if (PINC & 0b00000100){
    //DUMPSLN("Color Interrupt");
  }
  else{
    //Serial.println(F("Other CInt"));
  }
}

