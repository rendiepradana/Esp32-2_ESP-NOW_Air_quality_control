#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_now.h>
#include <WiFi.h>

const int LedBoard1 = 26;
const int LedBoard2 = 27;

float Smoke;
float Co2;

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C LCD address 0x27, size 16x2

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  float dataSensor;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;

// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].dataSensor = myData.dataSensor;

  // LED notification based on board ID
  if (myData.id == 1) {
    digitalWrite(LedBoard1, HIGH);  // Turn on LED for Board 1
    delay(500);
    digitalWrite(LedBoard2, LOW);   // Turn off LED for Board 1
  } if (myData.id == 2) {
    digitalWrite(LedBoard2, HIGH);  // Turn on LED for Board 2
    delay(500);
    digitalWrite(LedBoard2, LOW);   // Turn off LED for Board 2
  } else {
    // If ID is not 1 or 2, turn off both LEDs
    digitalWrite(LedBoard1, LOW);
    digitalWrite(LedBoard2, LOW);
  }

}

void setup() {
  Serial.begin(115200);

  pinMode(LedBoard1, OUTPUT);
  pinMode(LedBoard2, OUTPUT);

  lcd.begin();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {

  Smoke = boardsStruct[0].dataSensor;
  Co2 = boardsStruct[1].dataSensor;

  Serial.print(Smoke);
  Serial.println(" ppm");
  Serial.print(Co2);
  Serial.println(" ppm");

  // Display data on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Asap : ");
  lcd.print(Smoke);
  lcd.print(" ppm");
  lcd.setCursor(0, 1);
  lcd.print("CO2  : ");
  lcd.print(Co2);
  lcd.print(" ppm");

  delay(1000);
}
