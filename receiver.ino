#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_BMP280.h>

#define CHECK_BIT(value, pos) (value & (1U<< pos))
#define SET_BIT(value, pos) (value |= (1U<< pos))
#define CLEAR_BIT(value, pos) (value &= (~(1U<< pos)))

#define ALLOW_MOTOR 0

#define SIGNAL_TIMEOUT 1000

#define BATT_PIN A1

#define servo1_pin 2
#define servo2_pin 3
#define servo3_pin 4
#define servo4_pin 5

#define THROTTLE_MAX 165


// setup radio pipe addresses for each sensor node
const byte nodeAddress[5] = {'N', 'O', 'D', 'E', '1'};

// simple integer array for remote node data, in the form [node_id, returned_count]
int remoteNodeData[2] = {1, 1,};

RF24 radio(9, 10);  //CSN and CE pins

// The sizeof this struct should not exceed 32 bytes
struct Received_data {
  byte control;
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
};
Received_data received_data;

struct Telemetry_data {
  int ALT;
  uint8_t BAT;
};
Telemetry_data telemetry_data;

uint8_t ch1_value = 0;
uint8_t ch2_value = 0;
uint8_t ch3_value = 0;
uint8_t ch4_value = 0;

//The atmostpheric pressure at sea level. This is set in the setup()
float seaLevelPressure = 0;

//Control register. Gets updated from the received_data
byte control = 0x00;

bool signalTimeout = 0;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


Adafruit_BMP280 bmp;

void setup()
{
  Serial.begin(9600);
  if (!bmp.begin()) {
    //Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //Set initial values for the channels
  received_data.ch1 = 0; //Throttle
  received_data.ch2 = 127;
  received_data.ch3 = 127;
  received_data.ch4 = 127;

  //Once again, begin and radio configuration
  radio.begin();
  delay(100);
  radio.setChannel(127);
  //Lowest data rate to increase range
  radio.setDataRate(RF24_250KBPS);
  //Maximum power amplifier levels to increase range
  radio.setPALevel(RF24_PA_MAX);
  // open a reading pipe on the chosen address - matches the master tx
  radio.openReadingPipe(1, nodeAddress);
  // enable ack payload - slave replies with data using this feature
  radio.enableAckPayload();
  // preload payload with initial data
  radio.writeAckPayload(1, &telemetry_data, sizeof(Telemetry_data));


  //We start the radio comunication
  radio.startListening();

  //Attach servos
  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);

  //Set the baseline for the atmospheric pressure used to determine the height.
  seaLevelPressure = bmp.readPressure() / 100;
  //Serial.println(seaLevelPressure);
}

void receive_the_data()
{
  static unsigned long lastReceived = millis();
  if ( radio.available() ) {
    radio.read(&received_data, sizeof(Received_data));
    lastReceived = millis();
    updateNodeData();
    signalTimeout = 0;
  }

  if (millis() - lastReceived > SIGNAL_TIMEOUT) {
    signalTimeout = true;
    //Serial.println(F("SIGNAL LOST"));
  }
}

void updateNodeData(void)
{
  telemetry_data.ALT = (int) bmp.readAltitude(seaLevelPressure);
  telemetry_data.BAT = analogRead(BATT_PIN) >> 2;

  //  Serial.print(telemetry_data.BAT);
  //  Serial.println(" bat");
  //  Serial.println(telemetry_data.ALT);
  //  Serial.println(" m");

  // set the ack payload ready for next request from the master device
  radio.writeAckPayload(1, &telemetry_data, sizeof(Telemetry_data));
}

void loop()
{
  receive_the_data();

  //Update control register
  control = received_data.control;

  static uint8_t prev_ch1_value = 0;
  ch1_value = map(received_data.ch1, 0, 255, 30, THROTTLE_MAX);
  //Serial.println(ch1_value);
  ch2_value = map(received_data.ch2, 0, 255, 15, 165);
  ch3_value = map(received_data.ch3, 0, 255, 15, 165);
  ch4_value = map(received_data.ch4, 0, 255, 15, 165);

  if (signalTimeout) {
    servo1.write(0);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);
  }
  else {
    if (CHECK_BIT(control, ALLOW_MOTOR)) {
      servo1.write(ch1_value);
    }
    else {
      servo1.write(0);
    }
    servo2.write(ch2_value);
    servo3.write(ch3_value);
    servo4.write(ch4_value);
  }
}
