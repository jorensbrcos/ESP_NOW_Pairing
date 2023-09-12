// Testing unsecure broadcast connection possibly to later implement a pairing between unknown devices
// Base
// 16.01.2023


#include <esp_now.h>
#include <WiFi.h>


bool pairing_response = false;
bool addressSet = false;
bool first_message = false;
bool peer_init = false;
bool pairing_done = false;
bool initialise_pairing = false;

//uint8_t pairAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t allAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t zzzAddress[6];

int buttonPin = 34;
int buttonStatus;

esp_now_peer_info_t tempPeer;
esp_now_peer_num_t peer_num;

// Must match the receiver structure
typedef struct pair_message_struct {
    bool pair_init = false; // must be unique for each sender board
    bool pair_response = false;
    int board_id; // different for every board
    float tt = 111;
    float hh = 111;
} pair_message_struct;

//Create a struct_message 
pair_message_struct pairing_message;

pair_message_struct board1;
pair_message_struct board2;
pair_message_struct board3;
pair_message_struct board4;
pair_message_struct board5;
pair_message_struct board6;
pair_message_struct board7;
pair_message_struct board8;
pair_message_struct boardStruct[8] = {board1,board2,board3,board4,board5,board6,board7,board8};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(buttonPin, INPUT);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  //esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_recv_cb(OnPairRecv);

  esp_now_register_send_cb(OnDataSent);  
}
 
void loop() {

  pairing();
  if (pairing_done == true){
    float Temp1 = boardStruct[0].tt;
    float Hum1 = boardStruct[0].hh;
    if(Temp1 != 111 && Hum1 != 111 ){
      Serial.print("  TEMP1 = ");
      Serial.print(Temp1);
      Serial.print("  HUM1 = ");
      Serial.print(Hum1);
    }
    float Temp2 = boardStruct[1].tt;
    float Hum2 = boardStruct[1].hh;
    if(Temp2 != 111 && Hum2 != 111 ){
      Serial.print("  TEMP2 = ");
      Serial.print(Temp2);
      Serial.print("  HUM2 = ");
      Serial.print(Hum2);
    } 
  }
  Serial.println(" ");
  delay(400);
}

void OnPairRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&pairing_message, incomingData, sizeof(pairing_message));
  // Update the structures with the new incoming data
  initialise_pairing = pairing_message.pair_init;
  memcpy(zzzAddress, mac_addr, 6);
  //esp_err_t bobo = esp_now_get_peer(mac_addr, tempPeer);
  boardStruct[pairing_message.board_id -1].tt = pairing_message.tt;
  boardStruct[pairing_message.board_id -1].hh = pairing_message.hh;   
}


void pairing(){
  buttonStatus = digitalRead(buttonPin);
  if(buttonStatus == 0){
    Serial.println("BUTPRESS");
    pairing_done = false;
    //Serial.print(pairing_done);
    while(pairing_done == false){
      //Serial.print(initialise_pairing);
      while(initialise_pairing == false){
        Serial.println("Waiting for pairing initilisation");
        delay(500);
        }
      if(initialise_pairing == true){
        bool check_mac = esp_now_is_peer_exist(zzzAddress);
        Serial.println(check_mac);  
//        if(bobo == ESP_OK) ){
//          Serial.println("Nosaka vai adrese ir sarakstā");
//          }
        if(check_mac == 0){
          // Register temporary peer
          memcpy(tempPeer.peer_addr, zzzAddress, 6);
          tempPeer.channel = 0;  
          tempPeer.encrypt = false;
  
          //peer_init = false;
          // Add peer        
          if (esp_now_add_peer(&tempPeer) != ESP_OK){
            Serial.println("Failed to add peer");
            //return;
            }
        }
        
      sendBackResponse(); 
      }     
     }
    buttonStatus = 1;
    }
  }

void sendBackResponse(){
  pairing_message.pair_response = true;
  pairing_message.pair_init = false;
  initialise_pairing = pairing_message.pair_init; //importanto
  esp_err_t sendStatus = esp_now_send(zzzAddress, (uint8_t *) &pairing_message, sizeof(pairing_message));
  if (sendStatus == ESP_OK) {
    pairing_done = true;
    Serial.println("Sent with success");
    }
  else {
    Serial.println("Error sending the data");
    }
  delay(500);
  pairing_message.pair_response = false;
  }
