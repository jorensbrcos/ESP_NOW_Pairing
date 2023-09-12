// Testing unsecure broadcast connection possibly to later implement a pairing between unknown devices
// NODE .   
// 16.01.2023 tests uz bredborda veel vienai platei


#define I2C_SDA 23
#define I2C_SCL 22

// Wireless transmission information
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "SparkFun_Si7021_Breakout_Library.h"

//esp_now_peer_info_t peerInfo;
esp_now_peer_num_t peer_num;
esp_now_peer_info_t baseInfo;

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // If this mac address used, then broadcasts to all
uint8_t allAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t baseAddress[6];

//bool response_rcv = false;
bool pairing_done = false;
bool pairing_response_rcv = false;

int buttonPin = 34;
int buttonStatus;

float TEMPF = 0;
float HUMF = 0;

//Create Instance of HTU21D or SI7021 temp and humidity sensor 
Weather sensor;

// Structure example to send data
// Must match the receiver structure
typedef struct pair_message_struct {
    bool pair_init = false; // must be unique for each sender board
    bool pair_response = false;
    int board_id = 2; // different for every board
    float tt = 111;
    float hh = 111;
} pair_message_struct;

//Create a struct_message 
pair_message_struct pairing_message;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

void OnPairRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&pairing_message, incomingData, sizeof(pairing_message));
  memcpy(baseAddress, mac_addr, 6);
  // Update the structures with the new incoming data
  pairing_response_rcv = pairing_message.pair_response;
  //response_rcv = pairing_message.respond;
  
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Wire.begin(I2C_SDA,I2C_SCL,100000);
  //Initialize the I2C sensors and ping them
  sensor.begin();
  
  pinMode(buttonPin, INPUT);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register temporary peer
  esp_now_peer_info_t tempPeer;
  memcpy(tempPeer.peer_addr, allAddress, 6);
  tempPeer.channel = 0;  
  tempPeer.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&tempPeer) != ESP_OK){
    Serial.println("Failed to add peer");
    //return;
  }
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  //esp_now_register_send_cb(OnDataSent);

  esp_now_register_recv_cb(OnPairRecv);
  
  
}
 
void loop() {

  buttonStatus = digitalRead(buttonPin);
  if(buttonStatus == 0){
    Serial.println("BUTPRESS");
    pairing_done = false;
    pairing();
    }

  //Receive temperature an humidity data from Si7021 sensor
  getWeather();
  printInfo();
  delay(100);

  esp_err_t sendStatus = esp_now_send(baseAddress, (uint8_t *) &pairing_message, sizeof(pairing_message));
}


void pairing() {
  while(pairing_done == false){
    pairing_message.pair_init = true;
    
    esp_err_t sendStatus = esp_now_send(allAddress, (uint8_t *) &pairing_message, sizeof(pairing_message));
  
    if (sendStatus == ESP_OK) {
      Serial.println("Sent with success");
      //Waiting for the response message
      while(pairing_response_rcv == false){
        Serial.println("Waiting for the response message");
        delay(500);
        buttonStatus = digitalRead(buttonPin);
        if(buttonStatus == 0){
          Serial.println("Exit response waiting");
          pairing_response_rcv = true;
          }
        }
        bool check_mac = esp_now_is_peer_exist(baseAddress);
        Serial.println(check_mac);  
        if(check_mac == 0){
          // Register peer
          memcpy(baseInfo.peer_addr, baseAddress, 6);
          baseInfo.channel = 0;  
          baseInfo.encrypt = false;
          // Add peer        
          if (esp_now_add_peer(&baseInfo) != ESP_OK){
            Serial.println("Failed to add peer");
            //return;
            }
        }
        pairing_done = true;
        Serial.println("Pairing done");
        pairing_response_rcv = false;
      }
    else {
      Serial.println("Error sending the data");
      }
    delay(500);
  }

}

void getWeather(){
  // Measure Relative Humidity from the Si7021
  HUMF = sensor.getRH();
  pairing_message.hh = HUMF;
  
  // Measure Temperature from the Si7021
  TEMPF = sensor.getTemp();
  pairing_message.tt = TEMPF;
}

void printInfo()
{
//This function prints the weather data out to the default Serial Port

  Serial.print("Temp:");
  Serial.print(TEMPF);
  Serial.print(" C, ");

  Serial.print("Humidity:");
  Serial.print(HUMF);
  Serial.println("%");
}
