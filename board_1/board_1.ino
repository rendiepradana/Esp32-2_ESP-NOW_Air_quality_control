#include <esp_now.h>
#include <WiFi.h>
#include <MQUnifiedsensor.h>

#define Board ("ESP 32")
#define Pin (34) // Pin analog untuk sensor MQ-2
#define Type ("MQ-2")
#define Voltage_Resolution (5)
#define ADC_Bit_Resolution (12)
#define RatioMQ2CleanAir (9.83)

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

const int LedPin = 26 ;

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0xFC, 0xB4, 0x67, 0x93, 0xFE, 0x88};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int id;       // must be unique for each sender board
  float dataSensor;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    Serial.println("Sent with success");
    digitalWrite(LedPin,HIGH);
  } else {
    Serial.println("Error sending the data");
    digitalWrite(LedPin,LOW);
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(LedPin, OUTPUT);

  MQ2.setRegressionMethod(1);
  MQ2.setA(36974);    // CO     | 36974  | -3.109
  MQ2.setB(-3.109);

  MQ2.init();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update();
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }

  MQ2.serialDebug(true);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Set values to send
  myData.id = 1;

  // Read smoke level from MQ-2 sensor
  MQ2.update();
  float sensorValue = MQ2.readSensor();
  myData.dataSensor = sensorValue;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  Serial.println("Data: ");
  Serial.println(sensorValue);

  

  delay(1000);
}
