#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce.h>

// display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // display address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// capacitancemeter constants
#define pin1mA 22
#define pin10uA 21
#define supplyVref3V 33
#define comparatorPullUp3V 15
#define comparatorOut 34
#define pinGround 40
#define dischargePin 20
#define buttonPin 2

// constant for current
const double current_1mA = 0.00095;
const double current_10uA = 9.0e-6;

// constant for cycle calibration
const uint64_t cycle10uA = 9522;
const uint64_t cycle1mA = 68599;

// stores the ARM cycle counter
uint64_t previousCycleCounter = 0;
uint64_t currentCycleCounter = 0;

// flags
bool buttonPressed = false;
bool capDetected = false;
bool checkTime = false;
bool switch1mA = false;

// button debounce
Bounce pushbutton = Bounce(buttonPin, 10);  // 10 ms debounce
byte previousState = HIGH;         // what state was the button last time
unsigned int count = 0;            // how many times has it changed to low
uint64_t countAt = 0;         // when count changed
unsigned int countPrinted = 0;     // last count printed


void setup() {
  // serial monitor for debug
  Serial.begin(9600);

  // input: comparator out & button
  pinMode(comparatorOut, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // output: vref supply, comparator vout, nmos
  pinMode(comparatorPullUp3V, OUTPUT);
  pinMode(supplyVref3V, OUTPUT);
  pinMode(pin1mA, OUTPUT);
  pinMode(pin10uA, OUTPUT);
  pinMode(pinGround, OUTPUT);
  // pinMode(dischargePin, OUTPUT);


  // initial values
  digitalWrite(comparatorPullUp3V, HIGH);
  digitalWrite(supplyVref3V, HIGH);
  digitalWrite(pinGround, LOW); // ground it at the beginning
  // digitalWrite(dischargePin, HIGH); // ground it at the beginning
  digitalWrite(pin10uA, HIGH);  // start with 10 uA
  digitalWrite(pin1mA, LOW);

  // setup the display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(2);       // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 20);     // Start at top-left corner
  display.cp437(true);          // Use full 256 char 'Code Page 437' font

  display.print("0.0000 uF");
  display.display();
}

void loop() {
  // listen for push button
  if (pushbutton.update()) {
    if (pushbutton.fallingEdge()) {
      count = count + 1;
      countAt = millis();
    }
  } else {
    // debounce check
    if (count != countPrinted) {
      uint64_t nowMillis = millis();
      if (nowMillis - countAt > 100) {
        // Serial.print("count: ");
        // Serial.println(count);
        countPrinted = count;

        buttonPressed = true;
      }
    }
  }

  // start measurement at 10uA when button pressed
  if (buttonPressed) {
    buttonPressed = false;
    checkTime = true;

    // attach RISING interrupt to the comparator output
    attachInterrupt(digitalPinToInterrupt(comparatorOut), comparatorISR, RISING);

    pinMode(pinGround, INPUT); // let it float to V-
    // digitalWrite(dischargePin, LOW); // float junction to V-
    previousCycleCounter = ARM_DWT_CYCCNT;
  }

  // if a capacitor has been detected
  if (capDetected) {
    capDetected = false;
    checkTime = false;

    // get the time
    uint64_t time = currentCycleCounter - previousCycleCounter;

    // calibrate
    uint64_t adjustedTime = time - cycle10uA;

    if (adjustedTime < 1203) {
      adjustedTime = time;
    }

    adjustedTime *= 1E9f/F_CPU; // time in ns

    // Serial.println(time);

    // calculate capacitance
    double capacitance = 0.0;

    if (switch1mA) {
      switch1mA = false;
      capacitance = adjustedTime * (current_1mA) / 1.002;
    } else {
      capacitance = adjustedTime * (current_10uA) / 1.002;
    }

    // display the result
    displayMeasurement(capacitance);

    // reset everything for next measurement
    digitalWrite(pin10uA, HIGH);  // start with 10 uA
    digitalWrite(pin1mA, LOW);
    
    // sleep for 1s
    delay(1000);
  }

  // keep checking the time to detect when to switch
  if (checkTime) {
    uint64_t currentTime = ARM_DWT_CYCCNT;
    currentTime -= previousCycleCounter;
    currentTime *= 1E9f/F_CPU; // time in ns

    // 10 ms & cap still not detected
    if (currentTime >= 10000000 && !capDetected && !switch1mA) {
      pinMode(pinGround, OUTPUT);
      digitalWrite(pinGround, LOW); // discharge
      
      // switch to 1mA
      digitalWrite(pin10uA, LOW);
      digitalWrite(pin1mA, HIGH);

      switch1mA = true;

      // wait 1s
      delay(1000);

      pinMode(pinGround, INPUT); // let it float to V-
      previousCycleCounter = ARM_DWT_CYCCNT;      
    }
  }
}

// handle the when capacitor is charged
void comparatorISR() {
  detachInterrupt(digitalPinToInterrupt(comparatorOut));
  currentCycleCounter = ARM_DWT_CYCCNT;

  capDetected = true;

  // ground immediately, don't let it go too high
  pinMode(pinGround, OUTPUT);
  digitalWrite(pinGround, LOW);
}

// write measurement to the display
void displayMeasurement(double capacitance) {
  // 0 = pf, 1 = nf, 2 = uf
  byte unit = 1;

  // pF check
  if (capacitance < 1) {
    capacitance *= 1e3;
    unit = 0;
  }

  // uF check
  else if (capacitance > 1e3) {
    capacitance /= 1e3;
    unit = 2;
  }

  // write to the display
  display.clearDisplay();
  display.setCursor(0, 20);

  // Resolution 3-1/2 digits (0000 - 1999)
  display.printf("%0.4f\n", capacitance);

  // set the location of the unit
  display.setCursor(75, 50);

  // select the correct unit
  switch(unit) {
    case 0:
      display.print(" pF");
    break;

    case 1:
      display.print(" nF");
    break;

    case 2:
      display.print(" uF");
    break;
  }

  display.display();
}