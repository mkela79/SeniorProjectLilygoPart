
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "ControllerNode.h"

#include "WiFi.h"
#include "ThingsBoard.h"

#define TINY_GSM_MODEM_SIM7070
#define TINY_GSM_RX_BUFFER 1024 
#define SerialAT Serial1

#include <TinyGsmClient.h>



#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define UART_BAUD           9600
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define in1 13
#define in2 14

#define in3 15
#define in4 2

#define PIN_ADC_BAT 34
#define PIN_ADC_SOLAR 35
#define ADC_BATTERY_LEVEL_SAMPLES 100

float referenceVoltage = 3.3;
int adcResolution = 4095;
float batteryVoltage;

#define MOVE_TIMEOUT 10000
#define MESURE_FREQ 50
#define MIN_DISTANCE 400

constexpr char WIFI_SSID[] = "Campus";
constexpr char WIFI_PASSWORD[] = "";


constexpr char TOKEN[] = "36PmOSPsCGtxDvsRq6fY";
constexpr char THINGSBOARD_SERVER[] = "thingsboard.cloud";

constexpr uint16_t THINGSBOARD_PORT = 1883;


WiFiClient client;
ThingsBoard tb(client);

const uint32_t TIMEOUT = 500UL;

uint16_t distance = 400;

unsigned long delay_movement;
unsigned long easy_task_movement;
int state = 0;
int pre_state = 0;

float lat,  lon;

SoftwareSerial softSerial(21, 22); // rx tx 
ControllerNode controllerNode(115200, &softSerial);

enum easyTaskStates {
  EASY_TASK_MOVE,
  EASY_TASK_MEASURE,
  EASY_TASK_RETRACT,
  EASY_TASK_STOP
};

enum moveDirections {
    MOVE_FORWARD = 1,
    MOVE_RIGHT = 2,
    MOVE_BACK = 3,
    MOVE_LEFT = 4
};

enum {
    SENSOR_RIGHT = 1,
    SENSOR_LEFT = 2,
    SENSOR_BACK = 3,
    SENSOR_FRONT = 4,
    SENSOR_HUMIDITY = 5
};


easyTaskStates easyState = EASY_TASK_MOVE;

void enableGPS(void)
{
    modem.sendAT("+CGPIO=0,48,1,1");
    if (modem.waitResponse(10000L) != 1) {
        Serial.println("Set GPS Power HIGH Failed");
    }
    modem.enableGPS();
}

void disableGPS(void)
{
    modem.sendAT("+CGPIO=0,48,1,0");
    if (modem.waitResponse(10000L) != 1) {
        Serial.println("Set GPS Power LOW Failed");
    }
    modem.disableGPS();
}

void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(1000);    
    digitalWrite(PWR_PIN, LOW);
}

void modemPowerOff()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(1500);   
    digitalWrite(PWR_PIN, LOW);
}


void modemRestart()
{
    modemPowerOff();
    delay(2000);
    modemPowerOn();
}

void getLanAndLong() {
  if (!modem.testAT()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting");
        modemRestart();
        return;
    }

    Serial.println("Start positioning . Make sure to locate outdoors.");
    Serial.println("The blue indicator light flashes to indicate positioning.");

    enableGPS();

    while (1) {
      Serial.println(modem.getGPSraw());
      int* a;
      modem.getGPSTime(a, a, a, a, a, a);
        if (modem.getGPS(&lat, &lon)) {
            Serial.println("The location has been locked, the latitude and longitude are:");
            Serial.print("latitude:"); Serial.println(lat);
            Serial.print("longitude:"); Serial.println(lon);
            break;
        }

        delay(2000);
    }

    disableGPS();

    Serial.println("/**********************************************************/");
    Serial.println("After the network test is complete, please enter the  ");
    Serial.println("AT command in the serial terminal.");
    Serial.println("/**********************************************************/\n\n");

}

void tryForward();
void tryBack();
void tryLeft();
void tryRight();

void read_adc_bat(float *voltage)
{
     *voltage = (analogRead(PIN_ADC_BAT) * referenceVoltage) / adcResolution  * 2.9;
}

void read_adc_solar(uint16_t *voltage)
{
    uint32_t in = 0;
    for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++) {
        in += (uint32_t)analogRead(PIN_ADC_SOLAR);
    }
    in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;

    uint16_t bat_mv = ((float)in / 4096) * 5000 * 2;

    *voltage = bat_mv;
}

void sendTelemetryData(float humidity, float longitude, float latitude, float batteryLevel) {
  if (!tb.connected()) {
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect to ThingsBoard server");
      return;
    }
  }

  Serial.print("Latitude: ");
  Serial.println(latitude);
  Serial.print("Longitude: ");
  Serial.println(longitude);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Battery Level: ");
  Serial.println(batteryLevel);

  tb.sendTelemetryFloat("moisture", humidity);
  tb.sendTelemetryFloat("soilMoisture", humidity);
  tb.sendTelemetryFloat("longitude", longitude);
  tb.sendTelemetryFloat("latitude", latitude);
  tb.sendTelemetryFloat("batteryLevel", batteryLevel);

  
  Serial.println("Telemetry data sent successfully.");
  Serial.println();
}

void moveForward() {
  // digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(in1, 193);
  analogWrite(in3, 200);
}

void moveBack() {
  // digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  // digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  analogWrite(in1, 293);
  analogWrite(in3, 300);
}

void moveLeft() {
  // digitalWrite(in1, HIGH);
  // digitalWrite(in2, LOW);
  // // digitalWrite(in3, HIGH);
  // digitalWrite(in4, LOW);
  // analogWrite(in1, 193);
  // analogWrite(in3, 200);
}

void moveRight() {
  // digitalWrite(in1, HIGH);
  // digitalWrite(in2, LOW);
  // // digitalWrite(in3, HIGH);
  // digitalWrite(in4, LOW);
  // analogWrite(in1, 193);
  // analogWrite(in3, 200);
}

void stop() {
  // digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(in1, LOW);
  analogWrite(in3, LOW);
}

void move_logic(moveDirections direction){ 
  if(state != pre_state){
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");

    switch (direction) {
      case MOVE_FORWARD:  state ? moveForward() : stop(); break;
      case MOVE_BACK:     state ? moveBack()    : stop(); break;
      case MOVE_LEFT:     state ? moveLeft()    : stop(); break;
      case MOVE_RIGHT:    state ? moveRight()   : stop(); break;
    }
    
    pre_state = state;
  }
}

void tryForward() {
  if(delay_movement + MESURE_FREQ < millis()) {
    delay_movement = millis();
    distance = controllerNode.readSensorData(SENSOR_FRONT, 50);
    Serial.print("for: ");
    Serial.println(distance);
    if(distance < 5) {
      return;
    }
    state = distance > MIN_DISTANCE;
    move_logic(MOVE_FORWARD);
  }
}

void tryBack() {
  if(delay_movement + MESURE_FREQ < millis()) {
      delay_movement = millis();
      distance = controllerNode.readSensorData(SENSOR_BACK, 50);
      Serial.print("back: ");
      Serial.println(distance);
      if(distance < 5) {
        return;
      }
      state = distance > MIN_DISTANCE;
      move_logic(MOVE_BACK);
    }
}

void tryLeft() {
  if(delay_movement + MESURE_FREQ < millis()) {
      delay_movement = millis();
      distance = controllerNode.readSensorData(SENSOR_LEFT, 20);
      Serial.println(distance);
      if(distance < 5) {
        return;
      }
      state = distance > MIN_DISTANCE;
      move_logic(MOVE_LEFT);
    }
}

void tryRight() {
  if(delay_movement + MESURE_FREQ < millis()) {
      delay_movement = millis();
      distance = controllerNode.readSensorData(SENSOR_RIGHT, 20);
      Serial.println(distance);
      if(distance < 5) {
        return;
      }
      state = distance > MIN_DISTANCE;
      move_logic(MOVE_RIGHT);
    }
}

void setup() {

  Serial.begin(115200);
  controllerNode.init();
  Serial.println("Wall_e Robot");
  
  pinMode(PIN_ADC_BAT, INPUT);
  pinMode(PIN_ADC_SOLAR, INPUT);


  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
  analogWrite(in1, LOW);
  analogWrite(in3, LOW);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  if (tb.connect(THINGSBOARD_SERVER, TOKEN)) {
    Serial.println("Connected to ThingsBoard for telemetry data.");
  }
  delay(2000);
  easy_task_movement = millis();

  modemPowerOn();
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  Serial.println("/**********************************************************/");
  Serial.println("To initialize the network test, please make sure your GPS");
  Serial.println("antenna has been connected to the GPS port on the board.");
  Serial.println("/**********************************************************/\n\n");

}

void loop() {

    switch (easyState) {
      case EASY_TASK_MOVE: {
        if(easy_task_movement + MOVE_TIMEOUT > millis()) {
            tryForward();
        } else {
          stop();
          easyState = EASY_TASK_MEASURE;
          distance = 0;
          Serial.println("<-----end task move----->");
        }
      } break;

      case EASY_TASK_MEASURE: {
        uint16_t hum = controllerNode.measureHumidity();
        read_adc_bat(&batteryVoltage);
        getLanAndLong();
        sendTelemetryData(hum/10.0, lon, lat, batteryVoltage/4.2*100);
        easyState = EASY_TASK_RETRACT;
        easy_task_movement = millis();
        Serial.println("<-----end task measure----->");
      } break;

      case EASY_TASK_RETRACT: {
        if(easy_task_movement + MOVE_TIMEOUT > millis()) {
            tryBack();
        } else {
          stop();
          easyState = EASY_TASK_MOVE;
          Serial.println("<-----end task retract----->");
          distance = 0;
          delay(1000);
          easy_task_movement = millis();
        }
      } break;
    }
}