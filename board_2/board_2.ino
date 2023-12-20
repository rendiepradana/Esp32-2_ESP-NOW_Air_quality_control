#include <ESP8266WiFi.h>
#include <espnow.h>
#include <MQUnifiedsensor.h>

#define placa "ESP 8266"
#define Voltage_Resolution 5
#define pin A0 // Analog input 0 of your Arduino
#define type "MQ-135" // MQ135
#define ADC_Bit_Resolution 10
#define RatioMQ135CleanAir 3.6 // RS / R0 = 3.6 ppm

// Declare Sensor
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

const int LedPin = 2;

// REPLACE WITH RECEIVER MAC Address FC:B4:67:93:FE:88
uint8_t broadcastAddress[] = {0xFC, 0xB4, 0x67, 0x93, 0xFE, 0x88};

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 2

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id;
    float dataSensor;
} struct_message;

// Create a struct_message called test to store variables to be sent
struct_message myData;

unsigned long lastTime = 0;
unsigned long timerDelay = 1000;

// Pin untuk sensor MQ-135
const int mq135Pin = A0;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("\r\nLast Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
    digitalWrite(LedPin,HIGH);
  } else {
    Serial.println("Delivery fail");
    digitalWrite(LedPin,LOW);
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  pinMode(LedPin, OUTPUT);

  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); // _PPM =  a*ratio^b
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration CO2      | 110.47 | -2.862

  MQ135.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ135.update(); // Update data, the Arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }
  /*****************************  MQ Calibration ********************************************/
  MQ135.serialDebug(true);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  // Once ESPNow is successfully init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  if ((millis() - lastTime) > timerDelay) {
    // Baca nilai dari sensor MQ-135
    MQ135.update(); // Update data, the Arduino will read the voltage from the analog pin

    float co2Level = MQ135.readSensor();

    // Set nilai untuk dikirim
    myData.id = BOARD_ID;
    myData.dataSensor = co2Level;

    Serial.println("Data: ");
    Serial.println(co2Level);

    // Kirim pesan melalui ESP-NOW
    esp_now_send(0, (uint8_t *)&myData, sizeof(myData));
    lastTime = millis();
  }
}
