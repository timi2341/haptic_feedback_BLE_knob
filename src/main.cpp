#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBleKeyboard.h>

BleKeyboard bleKeyboard;
/*
const unsigned char epd_bitmap_knob_menu [] PROGMEM = { 
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc2,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24,
  0x40, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x40, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x40, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x40, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x40, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x40, 0x0c, 0x00, 0xf0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x40, 0x18, 0x01, 0x98, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x40, 0x30, 0x01, 0x99, 0xf0, 0xf8, 0xc3, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x40, 0x60, 0x01, 0x99, 0x99, 0x98, 0xc6, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x40, 0xc0, 0x01, 0x99, 0x99, 0x98, 0xc6, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x41, 0x80, 0x01, 0xf9, 0x99, 0x98, 0xc7, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x43, 0x40, 0x01, 0x99, 0x99, 0x98, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x46, 0x20, 0x01, 0x99, 0x98, 0xf8, 0xc6, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x4e, 0x20, 0x01, 0x99, 0x98, 0x18, 0x63, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x4f, 0xff, 0xf0, 0x00, 0x01, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16,
  0x40, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26,
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x21, 0x10, 0x02, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x21, 0x11, 0xc7, 0x8e, 0x3c, 0x78, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x08, 0x21, 0x12, 0x22, 0x11, 0x22, 0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x08, 0x21, 0x12, 0x22, 0x11, 0x22, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x28, 0xa1, 0x13, 0xe2, 0x1f, 0x22, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xaa, 0xa1, 0x12, 0x02, 0x10, 0x22, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x02, 0xaa, 0xa1, 0x12, 0x22, 0x11, 0x22, 0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0a, 0xaa, 0xa1, 0xe1, 0xc1, 0x8e, 0x22, 0x18, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x0a, 0xaa, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1e, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x21, 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x3f, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x21, 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x21, 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x21, 0x08, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x21, 0x08, 0x43, 0x80, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x21, 0x08, 0x44, 0x44, 0x00, 0x00, 0x00, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x21, 0x08, 0x44, 0x0f, 0x26, 0x38, 0xf0, 0xf3, 0xcf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x21, 0x08, 0x44, 0x04, 0x28, 0x44, 0x89, 0x11, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x21, 0x08, 0x43, 0x84, 0x30, 0x44, 0x89, 0x11, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x20, 0xf0, 0x40, 0x44, 0x20, 0x7c, 0x89, 0x11, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x20, 0x00, 0x40, 0x44, 0x20, 0x40, 0x89, 0x11, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x10, 0x00, 0x84, 0x44, 0x20, 0x44, 0x88, 0xf1, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x08, 0x01, 0x03, 0x83, 0x20, 0x38, 0x88, 0x10, 0xc8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x07, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
*/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- motor + sensor (same pins as original but use Wire on 9/8) --------------------
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 4);

// -------------------- default haptic params (kept from original) --------------------
float right_limit = 6.14;
float left_limit = 3.0;
float num_detents = 16;
float vel_p_value = 1.0f;
float vel_p_value_in_limits = 0.45f;
float vel_i_value = 0.0f;
float vel_d_value = 0.00f;
float pos_p_value = 30.0f;
float tf_value = 4.0f;
float threshold = 0.35f;
int prev_step = -999;

// commander for CLI (kept)
Commander command = Commander(Serial);
void changeNumOfDetents(char* cmd) { command.scalar(&num_detents, cmd); Serial.println("Changed num of detents"); }
void changePVal(char* cmd) { command.scalar(&vel_p_value, cmd); Serial.println("changed P Parameter"); }
void changeIVal(char* cmd) { command.scalar(&vel_i_value, cmd); Serial.println("changed I Parameter"); }
void changeDVal(char* cmd) { command.scalar(&vel_d_value, cmd); Serial.println("changed D Parameter"); }
void changePosPVal(char* cmd) { command.scalar(&pos_p_value, cmd); Serial.println("changed POS P Parameter"); }
void changeFVal(char* cmd) { command.scalar(&tf_value, cmd); Serial.println("changed Time filter constant"); }
void changeTh(char* cmd) { command.scalar(&threshold, cmd); Serial.println("changed threshold"); }
void changePLVal(char* cmd) { command.scalar(&vel_p_value_in_limits, cmd); Serial.println("changed P Within Limits Parameter"); }

// -------------------- encoder pins (mechanical HW-040) --------------------
const uint8_t ENC_A_PIN = 2; // encoder A
const uint8_t ENC_B_PIN = 3; // encoder B
const uint8_t ENC_BTN_PIN = 1; // encoder button (user wiring) - note: GPIO1 is UART TX; if unstable, consider moving

// encoder state helpers
volatile int enc_raw_accum = 0; // accumulate raw transitions
uint8_t last_enc_state = 0;

// button debounce
unsigned long last_btn_time = 0;
bool last_btn_level = HIGH;

// submenu navigation
enum TopMode { MODE_VOLUME = 0, MODE_SCROLL = 1, MODE_SNAP = 2, MODE_BRIGHT = 3, MODE_EXIT = 4 };
TopMode currentMode = MODE_VOLUME;
int mainMenuCursor = 0;
bool inSubmenu = false;
int submenu_index = 0;

// per-mode params (individual items editable)
struct Mode1_Params { float detent_pos_p; int detents; float threshold; float solid_p; int right_quads; } m1;
struct Mode2_Params { int detent_density; float detent_p; float vel_p; float pos_p; } m2;
struct Mode3_Params { float stiffness; float sensitivity; } m3;
struct Mode4_Params { float detent_pos_p; int detents; float threshold; float solid_p; int right_quads; } m4;

void initModeParams() {
  m1.detent_pos_p = pos_p_value;
  m1.detents = (int)num_detents;
  m1.threshold = threshold;
  m1.solid_p = 1.0f;
  m1.right_quads = 0;

  m2.detent_density = 12;
  m2.detent_p = 20.0f;
  m2.vel_p = 1.0f;
  m2.pos_p = pos_p_value;

  m3.stiffness = 8.0f;
  m3.sensitivity = 0.3f;

  m4.detent_pos_p = pos_p_value;
  m4.detents = (int)num_detents;
  m4.threshold = threshold;
  m4.solid_p = 8.0f;
  m4.right_quads = 0;
}

// -------------------- helper prototypes --------------------
void setupEncoderPins();
void pollEncoder();
bool buttonPressed();
void drawMainMenu();
void drawSubmenu(TopMode m);

// -------------------- BLE helpers --------------------
void sendVolumeUp()   { bleKeyboard.write(KEY_MEDIA_VOLUME_UP); }
void sendVolumeDown() { bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN); }
void sendNextTrack()  { bleKeyboard.write(KEY_MEDIA_NEXT_TRACK); }
void sendPrevTrack()  { bleKeyboard.write(KEY_MEDIA_PREVIOUS_TRACK); }
void sendBrightnessUp() {
  bleKeyboard.write(KEY_MEDIA_BRIGHTNESS_UP);
}
void sendBrightnessDown() {
  bleKeyboard.write(KEY_MEDIA_BRIGHTNESS_DOWN);
}

// -------------------- setup --------------------
void setup() {
  Serial.begin(115200);
  bleKeyboard.begin();

  // Initialize single I2C bus on SDA=9, SCL=8 (per your wiring)
  // Use 400kHz for AS5600
  Wire.begin(/*sda=*/9, /*scl=*/8, /*frequency=*/400000);

  // init display (uses Wire)
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed");
    for (;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.clearDisplay();

  // draw initial bitmap welcome
  // display.drawBitmap(0, 0, epd_bitmap_knob_menu, 128, 64, SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Hello! Starting Setup");
  display.display();

  // init sensor & motor AFTER Wire started
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  motor.velocity_limit = 3;
  motor.useMonitoring(Serial);
  motor.init();

  // init FOC - must be after sensor and motor.init()
  motor.initFOC();

  // add commander CLI as before
  command.add('s', changePosPVal, "POS P Value");
  command.add('t', changeNumOfDetents, "Number of detents");
  command.add('p', changePVal, "P Value");
  command.add('o', changePLVal, "P Value within limits");
  command.add('i', changeIVal, "I Value");
  command.add('d', changeDVal, "D Value");
  command.add('f', changeFVal, "Tf Value");
  command.add('h', changeTh, "Threshold Value");

  // encoder pins
  setupEncoderPins();
  initModeParams();

  // initial UI
  drawMainMenu();

  Serial.println(F("Setup complete. Motor ready."));
  delay(500);
}

// -------------------- encoder pin definitions
void setupEncoderPins() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  last_enc_state = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);
}

// lightweight poll - call frequently from loop
void pollEncoder() {
  uint8_t s = (digitalRead(ENC_A_PIN) << 1) | digitalRead(ENC_B_PIN);
  if (s != last_enc_state) {
    // Simple quadrature decode (increment by +1 or -1 per transition)
    int8_t change = 0;
    // two-bit gray transitions; map expected forward/back transitions
    if (last_enc_state == 0b00 && s == 0b01) change = 1;
    else if (last_enc_state == 0b01 && s == 0b11) change = 1;
    else if (last_enc_state == 0b11 && s == 0b10) change = 1;
    else if (last_enc_state == 0b10 && s == 0b00) change = 1;
    else if (last_enc_state == 0b00 && s == 0b10) change = -1;
    else if (last_enc_state == 0b10 && s == 0b11) change = -1;
    else if (last_enc_state == 0b11 && s == 0b01) change = -1;
    else if (last_enc_state == 0b01 && s == 0b00) change = -1;
    // accumulate raw transitions; we'll treat 2 transitions == 1 detent for HW-040
    enc_raw_accum += change;
    last_enc_state = s;
  }
}

// debounced button press returning true on falling edge (pressed LOW)
bool buttonPressed() {
  static bool latched = false;
  bool level = digitalRead(ENC_BTN_PIN);
  unsigned long now = millis();

  if (level != last_btn_level) {
    last_btn_time = now;
    last_btn_level = level;
  }

  if ((now - last_btn_time) > 40) {   // stable
    if (level == LOW && !latched) {
      latched = true;                 // latch until release
      return true;                    // single trigger
    }
    if (level == HIGH) {
      latched = false;                // release → unlock
    }
  }

  return false;
}


// -------------------- UI drawing --------------------
void drawMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Knob Modes"));
  display.println();

  const char* items[] = {"1 Bounded Vol", "2 Unbounded Scr", "3 Snap Middle", "4 Bounded Brt", "Exit"};
  for (int i=0;i<5;i++){
    if (i == mainMenuCursor) display.print("> ");
    else display.print("  ");
    display.println(items[i]);
  }
  display.display();
}

// helper to print float with fixed precision
String fstr(float v, int prec=2) {
  char buf[16];
  dtostrf(v, 0, prec, buf);
  return String(buf);
}

void drawSubmenu(TopMode m) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  switch(m) {
    case MODE_VOLUME:
      display.println(F("1:Limited Vol Control"));
      display.println();
      // items: 0 detent_pos_p, 1 detents, 2 threshold, 3 solid_p, 4 right_quads, 5 EXIT
      display.print(submenu_index==0?"> ":"  "); display.print(F("Det. Start Str: ")); display.println((int)m1.detent_pos_p);
      display.print(submenu_index==1?"> ":"  "); display.print(F("Num of detents: ")); display.println(m1.detents);
      display.print(submenu_index==2?"> ":"  "); display.print(F("Deadzone size: ")); display.println(fstr(m1.threshold,2));
      display.print(submenu_index==3?"> ":"  "); display.print(F("Det End Str: ")); display.println(fstr(m1.solid_p,1));
      display.print(submenu_index==4?"> ":"  "); display.print(F("Range: ")); display.println(m1.right_quads);
      display.print(submenu_index==5?"> ":"  "); display.println(F("EXIT"));
      break;
    case MODE_SCROLL:
      display.println(F("Mode2: Scroll Wheel"));
      display.println();
      // items: 0 detent_density,1 detent_p,2 vel_p,3 EXIT
      display.print(submenu_index==0?"> ":"  "); display.print(F("Detent Density: ")); display.println(m2.detent_density);
      display.print(submenu_index==1?"> ":"  "); display.print(F("Detent P: ")); display.println(fstr(m2.detent_p,1));
      display.print(submenu_index==2?"> ":"  "); display.print(F("Vel P: ")); display.println(fstr(m2.vel_p,2));
      display.print(submenu_index==3?"> ":"  "); display.print(F("Pos P: ")); display.println(fstr(m2.pos_p,2));
      display.print(submenu_index==4?"> ":"  "); display.println(F("EXIT"));
      break;
    case MODE_SNAP:
      display.println(F("Mode3: Snap Middle"));
      display.println();
      // items: 0 stiffness, 1 sensitivity, 2 EXIT
      display.print(submenu_index==0?"> ":"  "); display.print(F("Stiffness: ")); display.println(fstr(m3.stiffness,1));
      display.print(submenu_index==1?"> ":"  "); display.print(F("Sensitivity: ")); display.println(fstr(m3.sensitivity,2));
      display.print(submenu_index==2?"> ":"  "); display.println(F("EXIT"));
      break;
    case MODE_BRIGHT:
      display.println(F("Mode4: Bounded Bright"));
      display.println();
      // items: 0 detent_pos_p,1 detents,2 threshold,3 solid_p,4 right_quads,5 EXIT
      display.print(submenu_index==0?"> ":"  "); display.print(F("Detent P: ")); display.println(fstr(m4.detent_pos_p,1));
      display.print(submenu_index==1?"> ":"  "); display.print(F("Detents: ")); display.println(m4.detents);
      display.print(submenu_index==2?"> ":"  "); display.print(F("Threshold: ")); display.println(fstr(m4.threshold,2));
      display.print(submenu_index==3?"> ":"  "); display.print(F("Solid P: ")); display.println(fstr(m4.solid_p,1));
      display.print(submenu_index==4?"> ":"  "); display.print(F("Range +90 *: ")); display.println(m4.right_quads);
      display.print(submenu_index==5?"> ":"  "); display.println(F("EXIT"));
      break;
    default:
      break;
  }
  display.display();
}

// -------------------- math helper --------------------
float angleDiff(float a, float b) {
  float diff = a - b;
  while (diff > PI) diff -= 2*PI;
  while (diff < -PI) diff += 2*PI;
  return diff;
}

// -------------------- runtime state for modes --------------------
float virtual_unbounded_position = 0.0f;
float prev_motor_angle = 0.0f;
bool snap_sent_right = false;
bool snap_sent_left = false;
long last_virtual_step_sent = 0;

// -------------------- main loop --------------------
void loop() {
  // poll encoder many times per loop
  pollEncoder();

  // process encoder raw accumulation: HW-040 ~ 2 transitions per detent
  // We act when abs(enc_raw_accum) >= 2 -> one detent step (signed).
  if (enc_raw_accum >= 2 || enc_raw_accum <= -2) {
    int steps = enc_raw_accum / 2; // integer division: e.g. 3/2 = 1 => OK
    enc_raw_accum -= steps * 2;    // consume those transitions
    // Use steps for UI or value change
    if (!inSubmenu) {
      // navigate main menu up/down
      mainMenuCursor = constrain(mainMenuCursor - steps, 0, 4); // invert direction if needed
      drawMainMenu();
    } else {
      // modify the currently selected submenu item based on currentMode & submenu_index
      switch (currentMode) {
        case MODE_VOLUME:
          // submenu items: 0 detent_pos_p, 1 detents, 2 threshold, 3 solid_p, 4 right_quads, 5 EXIT
          if (submenu_index == 0) { m1.detent_pos_p = constrain(m1.detent_pos_p + steps * 1.0f, 0.0f, 30.0f); pos_p_value = m1.detent_pos_p; }
          else if (submenu_index == 1) { m1.detents = constrain(m1.detents + steps, 1, 128); num_detents = m1.detents; }
          else if (submenu_index == 2) { m1.threshold = constrain(m1.threshold + steps * 0.05f, 0.0f, 1.0f); threshold = m1.threshold; }
          else if (submenu_index == 3) { m1.solid_p = constrain(m1.solid_p + steps * 0.1f, 0.0f, 1.5f); vel_p_value_in_limits = m1.solid_p; }
          else if (submenu_index == 4) { m1.right_quads = constrain(m1.right_quads + steps, 0, 3); }
          // exit (5) not editable
          drawSubmenu(MODE_VOLUME);
          break;
        case MODE_SCROLL:
          // items: 0 density,1 detent_p,2 vel_p,3 EXIT
          if (submenu_index == 0) { m2.detent_density = constrain(m2.detent_density + steps, 1, 360); }
          else if (submenu_index == 1) { m2.detent_p = constrain(m2.detent_p + steps * 1.0f, 0.0f, 200.0f); }
          else if (submenu_index == 2) { m2.vel_p = constrain(m2.vel_p + steps * 0.05f, 0.0f, 2.0f); vel_p_value = m2.vel_p; }
          else if (submenu_index == 3) { m2.pos_p = constrain(m2.pos_p + steps * 0.5f, 0.0f, 100.0f); pos_p_value = m2.vel_p; }
          drawSubmenu(MODE_SCROLL);
          break;
        case MODE_SNAP:
          // items: 0 stiffness,1 sensitivity,2 EXIT
          if (submenu_index == 0) { m3.stiffness = constrain(m3.stiffness + steps * 0.5f, 0.0f, 200.0f); }
          else if (submenu_index == 1) { m3.sensitivity = constrain(m3.sensitivity + steps * 0.01f, 0.01f, 2.0f); }
          drawSubmenu(MODE_SNAP);
          break;
        case MODE_BRIGHT:
          // items: 0 detent_pos_p,1 detents,2 threshold,3 solid_p,4 right_quads,5 EXIT
          if (submenu_index == 0) { m4.detent_pos_p = constrain(m4.detent_pos_p + steps * 1.0f, 0.0f, 200.0f); pos_p_value = m4.detent_pos_p; }
          else if (submenu_index == 1) { m4.detents = constrain(m4.detents + steps, 1, 128); num_detents = m4.detents; }
          else if (submenu_index == 2) { m4.threshold = constrain(m4.threshold + steps * 0.05f, 0.0f, 5.0f); threshold = m4.threshold; }
          else if (submenu_index == 3) { m4.solid_p = constrain(m4.solid_p + steps * 0.5f, 0.0f, 200.0f); vel_p_value_in_limits = m4.solid_p; }
          else if (submenu_index == 4) { m4.right_quads = constrain(m4.right_quads + steps, 0, 3); }
          drawSubmenu(MODE_BRIGHT);
          break;
        default:
          break;
      }
    }
  }

  // handle encoder button press: advance submenu item (or enter submenu)
  if (buttonPressed()) {
    if (!inSubmenu) {
      // Enter submenu (unless Exit)
      if (mainMenuCursor == 4) {
        // Exit selected: do nothing / maybe go to default
        mainMenuCursor = 0;
        drawMainMenu();
      } else {
        inSubmenu = true;
        currentMode = (TopMode)mainMenuCursor;
        submenu_index = 0;
        drawSubmenu(currentMode);
      }
    } else {
      // In submenu: advance the submenu index; if hits EXIT index -> leave
      int itemsCount = 1;
      switch (currentMode) {
        case MODE_VOLUME: itemsCount = 6; break; // 5 items + EXIT
        case MODE_SCROLL: itemsCount = 5; break;
        case MODE_SNAP: itemsCount = 3; break;
        case MODE_BRIGHT: itemsCount = 6; break;
        default: itemsCount = 1; break;
      }
      submenu_index++;
      if (submenu_index >= itemsCount) {
        // apply final param mappings when leaving submenu
        if (currentMode == MODE_VOLUME) {
          pos_p_value = m1.detent_pos_p; num_detents = m1.detents; threshold = m1.threshold; vel_p_value_in_limits = m1.solid_p;
        } else if (currentMode == MODE_SCROLL) {
          vel_p_value_in_limits = m2.pos_p;
          vel_p_value = m2.vel_p;
        } else if (currentMode == MODE_SNAP) {
          // nothing to sync globally except params already in m3
        } else if (currentMode == MODE_BRIGHT) {
          pos_p_value = m4.detent_pos_p; num_detents = m4.detents; threshold = m4.threshold; vel_p_value_in_limits = m4.solid_p;
        }
        // leave submenu
        inSubmenu = false;
        drawMainMenu();
      } else {
        drawSubmenu(currentMode);
      }
    }
    // small debounce delay to avoid accidental multi-step from press
    // delay(120);
  }

  // ---- Motor/FOC update and per-mode behavior ----
  // update PID values
  motor.PID_velocity.I = vel_i_value;
  motor.PID_velocity.D = vel_d_value;
  motor.LPF_velocity.Tf = tf_value;

  float motor_angle = motor.shaftAngle();
  float nearest_angle = motor_angle; // default target

  switch (currentMode) {
    case MODE_VOLUME: {
      float q90 = (PI/2.0f) * (float)m1.right_quads;
      float mode_right_limit = right_limit + q90;
      if (m1.detents <= 1) {
        nearest_angle = left_limit;
      } else if (motor_angle < left_limit) {
        nearest_angle = left_limit;
      } else if (motor_angle > mode_right_limit) {
        nearest_angle = mode_right_limit;
      } else {
        float range = mode_right_limit - left_limit;
        float step_size = range / (float)m1.detents;
        float current_step = round((motor_angle - left_limit) / step_size);
        int current_step_int = (int)current_step;
        if (prev_step != current_step_int) {
          if (prev_step < current_step_int) sendVolumeDown();
          else sendVolumeUp();
        }
        prev_step = current_step_int;
        nearest_angle = left_limit + (current_step * step_size);
      }
      // haptic feel
      if (motor_angle < left_limit - m1.threshold || motor_angle > mode_right_limit + m1.threshold) {
        motor.PID_velocity.P = vel_p_value;
        motor.P_angle.P = m1.detent_pos_p;
      } else if (motor_angle < left_limit || motor_angle > mode_right_limit) {
        motor.P_angle.P = 0.1;
        motor.PID_velocity.P = 1;
      } else {
        motor.PID_velocity.P = vel_p_value_in_limits;
        motor.P_angle.P = m1.detent_pos_p;
      }
      break;
    }

    case MODE_SCROLL: {
      // accumulate delta steps from angle differences
      float d_ang = angleDiff(motor_angle, prev_motor_angle);
      prev_motor_angle = motor_angle;
      float steps_per_rev = (float)m2.detent_density;
      float step_per_rad = steps_per_rev / (2.0f * PI);
      float d_steps = d_ang * step_per_rad;
      virtual_unbounded_position += d_steps;
      long virtual_step = (long)round(virtual_unbounded_position);
      if (virtual_step != last_virtual_step_sent) {
        if (virtual_step > last_virtual_step_sent) {
          bleKeyboard.write(KEY_PAGE_UP);
        } else {
          bleKeyboard.write(KEY_PAGE_DOWN);
        }
        last_virtual_step_sent = virtual_step;
      }
      // snap target to virtual detent angle to provide haptic detents
      float detent_angle = (2.0f * PI) / steps_per_rev;
      nearest_angle = motor_angle - fmod(motor_angle, detent_angle);
      motor.P_angle.P = m2.detent_p;
      motor.PID_velocity.P = m2.vel_p;
      break;
    }

    case MODE_SNAP: {
      float center = (left_limit + right_limit) * 0.5f;
      float dfc = angleDiff(motor_angle, center);
      float absdfc = fabs(dfc);
      if (dfc > m3.sensitivity && !snap_sent_right) {
        sendNextTrack();
        snap_sent_right = true; snap_sent_left = false;
      }
      if (dfc < -m3.sensitivity && !snap_sent_left) {
        sendPrevTrack();
        snap_sent_left = true; snap_sent_right = false;
      }
      if (absdfc < (m3.sensitivity * 0.5f)) { snap_sent_left = snap_sent_right = false; }
      if (absdfc < (m3.sensitivity * 1.5f)) {
        nearest_angle = center;
        motor.P_angle.P = m3.stiffness * 4.0f;
        motor.PID_velocity.P = vel_p_value_in_limits;
      } else {
        nearest_angle = motor_angle;
        motor.P_angle.P = m3.stiffness;
        motor.PID_velocity.P = vel_p_value;
      }
      break;
    }

    case MODE_BRIGHT: {
      float q90 = (PI/2.0f) * (float)m4.right_quads;
      float mode_right_limit = right_limit + q90;
      if (m4.detents <= 1) {
        nearest_angle = left_limit;
      } else if (motor_angle < left_limit) {
        nearest_angle = left_limit;
      } else if (motor_angle > mode_right_limit) {
        nearest_angle = mode_right_limit;
      } else {
        float range = mode_right_limit - left_limit;
        float step_size = range / (float)m4.detents;
        float current_step = round((motor_angle - left_limit) / step_size);
        int current_step_int = (int)current_step;
        if (prev_step != current_step_int) {
          if (prev_step < current_step_int) sendBrightnessDown();
          else sendBrightnessUp();
        }
        prev_step = current_step_int;
        nearest_angle = left_limit + (current_step * step_size);
      }
      if (motor_angle < left_limit - m4.threshold || motor_angle > mode_right_limit + m4.threshold) {
        motor.PID_velocity.P = vel_p_value;
        motor.P_angle.P = m4.detent_pos_p;
      } else if (motor_angle < left_limit || motor_angle > mode_right_limit) {
        motor.P_angle.P = 0.1;
        motor.PID_velocity.P = 1;
      } else {
        motor.PID_velocity.P = vel_p_value_in_limits;
        motor.P_angle.P = m4.detent_pos_p;
      }
      break;
    }

    default:
      nearest_angle = motor_angle;
      break;
  }

  motor.loopFOC();
  motor.move(nearest_angle);
  command.run();

  // small delay to keep loop stable and give time to other tasks
  // delay(4);
}
