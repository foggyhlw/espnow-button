
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <WS2812FX.h>

// uint8_t broadcastAddress[] = {0x84,0x0D,0x8E,0x81,0xD5,0x26};   //nodemcu
uint8_t broadcastAddress[] = {0x5c, 0xcf, 0x7f, 0xb1, 0xf3, 0x1a};  // esp01s 
// uint8_t broadcastAddress[] = {0x48, 0x3f, 0xda, 0x9e, 0x39, 0x6a};  //light board

#define LED_COUNT 1
#define LED_PIN 4
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
#define BUTTON1 12
#define BUTTON2 14
#define BUTTON3 13
// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char mac[32];
  int buttonNumber;
  float batteryVoltage;
} struct_message;

// float batteryVoltage = 0;
// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  // Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    // Serial.println("Delivery success");
  }
  else{
    // Serial.println("Delivery fail");
  }
}
 
void setup() {
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  // Init Serial Monitor
  // Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  strcpy(myData.mac, WiFi.macAddress().c_str());
    // strcpy(myData.mac,"mac1");

    if(digitalRead(BUTTON1) == LOW){
      myData.buttonNumber = BUTTON1;
    }
    if(digitalRead(BUTTON2) == LOW){
      myData.buttonNumber = BUTTON2;
    }
    if(digitalRead(BUTTON3) == LOW){
      myData.buttonNumber = BUTTON3;
    }

    myData.batteryVoltage = analogRead(A0)*5.6/1023;
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));


    ws2812fx.init();
    ws2812fx.setBrightness(50);

    if (myData.batteryVoltage < 3.9){
       ws2812fx.setColor(RED);
    }
    else{
       ws2812fx.setColor(GREEN);
    }
    ws2812fx.start();
}
 
void loop() {
  ws2812fx.service();
}
