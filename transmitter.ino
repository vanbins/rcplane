#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CHECK_BIT(value, pos) bool(value & (1U<< pos))
#define SET_BIT(value, pos) (value |= (1U<< pos))
#define CLEAR_BIT(value, pos) (value &= (~(1U<< pos)))
#define TOGGLE_BIT(value, pos) (value ^= (1UL << pos))

#define ALLOW_MOTOR 0

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Radio
#define CE  9
#define CSN 10

//Analog joystick inputs
#define LEFT_VERTICAL     A6
#define LEFT_HORIZONTAL   A7
#define RIGHT_VERTICAL    A2
#define RIGHT_HORIZONTAL  A3

//Joystick button inputs
#define LEFT_BTN  7
#define RIGHT_BTN 8

//Potentiometers
#define POT_1 A0
#define POT_2 A1

//Switches
#define SW_1 4
#define SW_2 3

//LEDs
#define LED_R 6
#define LED_L 5

//Button debounce time in ms
#define DEBOUNCE 20

//Altitude at which warning (LED_L) comes on
#define MAX_ALT 100

//Time before signal is considered timed out
#define SIGNAL_TIMEOUT 500

const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL; //Remember that this code should be the same for the receiver

// setup radio pipe address for remote sensor node
const byte nodeAddress[5] = {'N', 'O', 'D', 'E', '1'};
// integer array for slave node data:[node_id, returned_count]
int remoteNodeData[2] = {1, 1,};

RF24 radio(CE, CSN);

// The sizeof this struct should not exceed 32 bytes
struct Data_to_be_sent {
  byte control;
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
};
Data_to_be_sent sent_data;

struct Telemetry_data {
  int ALT;
  uint8_t BAT;
};
Telemetry_data telemetry_data;

//Menu navigators
uint8_t page;
uint8_t menuitem;

//Last time a button was pressed
unsigned long last_press = millis();

//Button states
bool SW_1_state;
bool SW_2_state;
bool LEFT_BTN_state;
bool RIGHT_BTN_state;
bool SW_1_state_prev      = false;
bool SW_2_state_prev      = false;
bool LEFT_BTN_state_prev  = false;
bool RIGHT_BTN_state_prev = false;

bool draw_trim = false;
int8_t ele_trim = 0, rud_trim = 0, ail_trim = 0;

float voltage; //Current battery voltage
float voltage_max = 13.2; //Voltage at which analogRead on the reveicer reads 255 (1023)
float voltage_threshold = 11.3; //Voltage below this will set off warning
//https://blog.ampow.com/lipo-voltage-chart/

byte control = 0x00;

bool signalTimeout = 0;///////////////////////////////////////

void setup()
{
  Serial.begin(9600);

  pinMode(SW_1, INPUT_PULLUP);
  pinMode(SW_2, INPUT_PULLUP);
  pinMode(LEFT_BTN, INPUT_PULLUP);
  pinMode(RIGHT_BTN, INPUT_PULLUP);

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);

  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_L, HIGH);

  //Radio initialize ******************************************
  radio.begin();
  delay(100);
  radio.setChannel(127);
  //Lowest data rate to increase range
  radio.setDataRate(RF24_250KBPS);
  //Maximum power amplifier levels to increase range
  radio.setPALevel(RF24_PA_MAX);
  // set time between retries and max no. of retries
  radio.setRetries(5, 5);
  // enable ackpayload - enables each slave to reply with data
  radio.enableAckPayload();
  // setup write pipe to remote node - must match node listen address
  radio.openWritingPipe(nodeAddress);

  //Menu initialize
  page = 0;
  menuitem = 0;

  //Attempt to initialize the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;); // Don't proceed, loop forever
  }

  display.display();
  delay(1000);

  // Clear the buffer
  display.clearDisplay();
  display.display();
}

void loop()
{
  static unsigned long lastReceived = millis();/////////////////////////////////////
  delay(2);
  voltage = telemetry_data.BAT * (voltage_max / 255);

    update_LEDs();

  if (draw_trim) {
    trim_menu(menuitem);
  }
  else {
    drawMenu();
  }
  sent_data.control = control;
  if (CHECK_BIT(control, ALLOW_MOTOR)) {
    sent_data.ch1 = map( max(analogRead(LEFT_VERTICAL), 520), 520, 1023, 0, 255);
  }
  else {
    sent_data.ch1 = 0;
  }
  sent_data.ch2 = max(min((analogRead(LEFT_HORIZONTAL) >> 2) + rud_trim, 255), 0);
  sent_data.ch3 = max(min((analogRead(RIGHT_VERTICAL) >> 2) + ele_trim, 255), 0);
  sent_data.ch4 = max(min((analogRead(RIGHT_HORIZONTAL) >> 2) + ail_trim, 255), 0);

  bool tx_sent = true;
  tx_sent = radio.write(&sent_data, sizeof(Data_to_be_sent));
  //Serial.print("tx");
  //Serial.println(tx_sent);
  //tx_sent = true;
  //Serial.println(tx_sent);
  //If tx success - receive and read smart-post ack reply
  if (tx_sent) {
    if (radio.isAckPayloadAvailable()) {
      // read ack payload and copy message to telemetry_data
      radio.read(&telemetry_data, sizeof(Telemetry_data));
      ///////////////////////////////
      lastReceived = millis();
      ///////////////////
    }
  }
  else {
    //Serial.println("[-] The transmission to the node failed.");
  }

  //check if signal is lost for predefined time
  if (millis() - lastReceived > SIGNAL_TIMEOUT) {
    signalTimeout = true;
    //Serial.println(F("SIGNAL LOST"));
  }
  else {
    signalTimeout = false;
  }
}

void drawMenu() {
  switch (page) {
    case 0:
      display.clearDisplay();
      display.setTextSize(2);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.setCursor(0, 0);     // Start at top-left corner
      display.cp437(true);         // Use full 256 char 'Code Page 437' font
      display.print(telemetry_data.ALT);
      display.println(F(" M"));
      display.println(voltage);
      //display.println(F(" V"));
      if (signalTimeout) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.println(F("!SIG"));
      }
      //      draw_AH();
      if (check_SW_1() && page < 2) {
        page++;
      } else if (check_SW_2() && page > 0) {
        page--;
      }
      break;

    case 1: //Trim menu
      display.clearDisplay();
      display.setTextSize(2);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.setCursor(0, 0);     // Start at top-left corner
      display.cp437(true);         // Use full 256 char 'Code Page 437' font
      display.println(F("TRIM"));
      if (menuitem == 1) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      }
      display.println(F("Elevator"));
      display.setTextColor(SSD1306_WHITE);

      if (menuitem == 2) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      }
      display.println(F("Rudder"));
      display.setTextColor(SSD1306_WHITE);

      if (menuitem == 3) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      }
      display.println(F("Ailerons"));

      //Button handling
      if (menuitem > 0) {
        if (check_SW_1() && menuitem < 3) {
          menuitem++;
        }
        else if (check_SW_2()) {
          menuitem--;
        }
        else if (check_RIGHT_BTN()) {
          trim_menu(menuitem);
        }
        else if (check_LEFT_BTN()) {
          menuitem = 0;
        }
      }

      else {
        if (check_SW_1() && page < 2) {
          page++;
        } else if (check_SW_2() && page > 0) {
          page--;
        } else if (check_RIGHT_BTN()) {
          menuitem = 1;
        } else if (check_LEFT_BTN()) {
          page = 0;
          menuitem = 0;
        }
      }
      break;

    case 2:
      display.clearDisplay();
      display.setTextSize(2);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.setCursor(0, 0);     // Start at top-left corner
      display.cp437(true);         // Use full 256 char 'Code Page 437' font
      display.println(F("SETTINGS"));

      if (menuitem == 1) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      }
      display.print(F("MOTOR: "));
      display.println(CHECK_BIT(control, ALLOW_MOTOR));
      display.setTextColor(SSD1306_WHITE); // Draw white text

      if (menuitem == 2) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      }
      display.print(F("i2: "));
      display.println(CHECK_BIT(control, 1));
      display.setTextColor(SSD1306_WHITE); // Draw white text

      if (menuitem == 3) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      }
      display.print(F("i3: "));
      display.println(CHECK_BIT(control, 2));
      display.setTextColor(SSD1306_WHITE); // Draw white text

      //      if (check_SW_1() && page < 2) {
      //        page++;
      //      } else if (check_SW_2() && page > 0) {
      //        page--;
      //      }

      //Button handling
      if (menuitem > 0) {
        if (check_SW_1() && menuitem < 3) {
          menuitem++;
        }
        else if (check_SW_2()) {
          menuitem--;
        }
        else if (check_RIGHT_BTN()) {
          TOGGLE_BIT(control, menuitem - 1);
        }
        else if (check_LEFT_BTN()) {
          menuitem = 0;
        }
      }

      else {
        if (check_SW_1() && page < 2) {
          page++;
        } else if (check_SW_2() && page > 0) {
          page--;
        } else if (check_RIGHT_BTN()) {
          menuitem = 1;
        } else if (check_LEFT_BTN()) {
          page = 0;
          menuitem = 0;
        }
      }
      break;
  }
  display.display();
}

void draw_AH() {
  int roll = 10;
  int pitch = 5;
  //display.drawRect(display.width()/2, 0, display.width()/2, display.height(), SSD1306_WHITE);
  //outer ring (roll)
  display.fillCircle(display.width() / 4 * 3, display.height() / 2, display.height() / 2 - 1, SSD1306_WHITE);
  display.fillTriangle(display.width() / 2, 0, display.width() / 2, display.height() / 2 - roll + pitch, display.width(), display.height() / 2 + roll + pitch, SSD1306_BLACK);
  display.fillTriangle(display.width() / 2, 0, display.width(), display.height() / 2 + roll + pitch, display.width(), 0, SSD1306_BLACK);

  //plane
  display.drawFastHLine(display.width() / 4 * 3 - 8, display.height() / 2 + 1, 5 , SSD1306_INVERSE);
  display.drawFastHLine(display.width() / 4 * 3 + 4, display.height() / 2 + 1, 5 , SSD1306_INVERSE);
  display.drawFastHLine(display.width() / 4 * 3 - 3, display.height() / 2 + 3, 6 , SSD1306_INVERSE);
  display.drawPixel(display.width() / 4 * 3, display.height() / 2 + 1, SSD1306_INVERSE);
  //outer outer ring edge (decoration)
  display.drawCircle(display.width() / 4 * 3, display.height() / 2, display.height() / 2 - 1, SSD1306_WHITE);
}

void settings_menu(uint8_t) {

}

void trim_menu(uint8_t menuitem) {
  draw_trim = true;
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.print(F("TRIM "));
  if (menuitem == 1) {
    if (check_SW_1()) {
      ele_trim++;
    }
    if (check_SW_2()) {
      ele_trim--;
    }
    display.println(F("ELE"));
    display.println(ele_trim);
    display.drawRect(display.width() - 16, 0, 10, display.height(), SSD1306_WHITE);
    display.drawFastHLine(display.width() - 20, display.height() / 2, 18, SSD1306_WHITE);
    display.fillRect(display.width() - 16, display.height() / 2 - ele_trim, 10, display.height() / 2 + ele_trim, SSD1306_WHITE);

  }
  else if (menuitem == 2) {
    if (check_SW_1()) {
      rud_trim--;
    }
    if (check_SW_2()) {
      rud_trim++;
    }
    display.println(F("RUD"));
    display.println(rud_trim);
    display.drawRect(0, display.height() - 16, display.width(), 10, SSD1306_WHITE);
    display.drawFastVLine(display.width() / 2 , display.height() - 20, 18, SSD1306_WHITE);
    display.drawFastVLine(display.width() / 2 + rud_trim , display.height() - 16, 10, SSD1306_WHITE);
  }
  else if (menuitem == 3) {
    if (check_SW_1()) {
      ail_trim--;
    }
    if (check_SW_2()) {
      ail_trim++;
    }
    display.println(F("AIL"));
    display.println(ail_trim);
    display.drawRect(0, display.height() - 30, display.width(), 30, SSD1306_WHITE);
    display.drawFastHLine(0, display.height() - 15, display.width(), SSD1306_WHITE);
    display.drawLine(0, display.height() - 15 + ail_trim, display.width(), display.height() - 15 - ail_trim, SSD1306_WHITE);
  }

  if (check_LEFT_BTN()) {
    draw_trim = false;
  }
  display.display();
}

void update_LEDs() {
  if (voltage < voltage_threshold) {
    digitalWrite(LED_R, LOW);
  }
  else {
    digitalWrite(LED_R, HIGH);
  }
  if (telemetry_data.ALT > MAX_ALT) {
    digitalWrite(LED_L, LOW);
  }
  else {
    digitalWrite(LED_L, HIGH);
  }
}

bool check_SW_1() {
  bool rv = false;
  if ((millis() - last_press) < DEBOUNCE) {
    return false;
  }
  SW_1_state = !digitalRead(SW_1);
  if (SW_1_state != SW_1_state_prev) {
    if (SW_1_state == true) {
      rv = true;
    }
    last_press = millis();
    SW_1_state_prev = SW_1_state;
    return rv;
  }
  else {
    return false;
  }
}

bool check_SW_2() {
  bool rv = false;
  if ((millis() - last_press) < DEBOUNCE) {
    return false;
  }
  SW_2_state = !digitalRead(SW_2);
  if (SW_2_state != SW_2_state_prev) {
    if (SW_2_state == true) {
      rv = true;
    }
    last_press = millis();
    SW_2_state_prev = SW_2_state;
    return rv;
  }
  else {
    return false;
  }
}

bool check_RIGHT_BTN() {
  bool rv = false;
  if ((millis() - last_press) < DEBOUNCE) {
    return false;
  }
  RIGHT_BTN_state = !digitalRead(RIGHT_BTN);
  if (RIGHT_BTN_state != RIGHT_BTN_state_prev) {
    if (RIGHT_BTN_state == true) {
      rv = true;
      Serial.println("R");
    }
    last_press = millis();
    RIGHT_BTN_state_prev = RIGHT_BTN_state;
    return rv;
  }
  else {
    return false;
  }
}

bool check_LEFT_BTN() {
  bool rv = false;
  if ((millis() - last_press) < DEBOUNCE) {
    return false;
  }
  LEFT_BTN_state = !digitalRead(LEFT_BTN);
  if (LEFT_BTN_state != LEFT_BTN_state_prev) {
    if (LEFT_BTN_state == true) {
      rv = true;
    }
    last_press = millis();
    LEFT_BTN_state_prev = LEFT_BTN_state;
    return rv;
  }
  else {
    return false;
  }
}
