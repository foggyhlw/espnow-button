#define MAP_100_2_255(val) map(val,0,100,0,56)   
//对应接收到的sender的gpio引脚号
#define BUTTON_NUMBER_1 13  // 12 is the number sent by esp-now sender, representing BUTTON_NUMBER_1
#define BUTTON_NUMBER_2 12 // 14 is the number sent by esp-now sender, representing BUTTON_NUMBER_2
#define BUTTON_NUMBER_3 14 // 13 is the number sent by esp-now sender, representing BUTTON_NUMBER_3

//本地io使用情况
#define INPUT_BUTTON_PIN 3  // local control button

#define PWM_PIN 2  // pin to control pwm
// #define DHT_PIN 4  
#define PWM_MIN 20
#define PWM_MAX 100
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "Button2.h"
// #include "TickTwo.h"
// #include "DHTesp.h"
#include <FadeLed.h>
// DHTesp dht;
Button2 button = Button2(INPUT_BUTTON_PIN);
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  char mac[32];
  int buttonNumber;
  float batteryVoltage;
} struct_message;

// Create a struct_message called myData
struct_message myData;
//int battery = 100;
bool power_state = true;
int pwm_duty_cycle = PWM_MIN;   //default duty cycle is 0
int pwm_step = 15;
FadeLed mainled(PWM_PIN);

// void ticker_cb();
// void dht_ticker_cb();

// TickTwo manual_timer(ticker_cb, 2000, 1);   // 1000ms delay ,1 time timer
// TickTwo dht_timer(dht_ticker_cb, 10000);  
// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("MAC: ");
  Serial.println(myData.mac);
  Serial.print("Button Nmumber: ");
  Serial.println(myData.buttonNumber);
  Serial.print("Battery Voltage: ");
  Serial.println(myData.batteryVoltage);
  //battery = myData.batteryVoltage;
  Serial.println();

  if (myData.buttonNumber == BUTTON_NUMBER_1){
    change_power_state();
  }
  if (myData.buttonNumber == BUTTON_NUMBER_2){
    step_up_pwm();
  }
  if (myData.buttonNumber == BUTTON_NUMBER_3){
    step_down_pwm();
  }
}
 
void setup() {
  FadeLed::setInterval(10);
  // pinMode(RELAY_POWER, OUTPUT);
  // pinMode(RELAY_LED, OUTPUT);
  // pinMode(PWM_PIN, OUTPUT);
  // analogWriteRange(100);
  // digitalWrite(RELAY_POWER, LOW);  //默认关闭
  // digitalWrite(RELAY_LED, LOW);
  mainled.setTime(200,true);
  mainled.set(MAP_100_2_255(PWM_MIN));  //开机最小亮度
  // dht.setup(DHT_PIN, DHTesp::DHT11); // Connect DHT sensor to GPIO5  D1
  // buzzer_setup(); 
  button.setClickHandler(single_click_handler);
  // button.setLongClickHandler(longClick_handler);
  // Initialize Serial Monitor
  Serial.begin(115200);
  // fastled_setup();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("\nMY MAC ADDRESS IS: ");
  Serial.print(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  // manual_timer.start();
  // dht_timer.start();
}


void loop() {
  button.loop();
  // manual_timer.update();
  // dht_timer.update();
  FadeLed::update();
  // buzzer_loop();
}


void change_power_state(){
if( power_state != 0 ){
    mainled.off();
  }
  else{
    mainled.set(MAP_100_2_255(pwm_duty_cycle)); 
  }
  power_state = !power_state;
}

void single_click_handler(Button2& btn){ 
  change_power_state();
  // manual_timer.start();
}

// void longClick_handler(Button2& btn){    // 长按关闭
//     digitalWrite(RELAY_LED, LOW);  
//     power_state = false;
//     analogWrite(PWM_PIN, 0);
//     digitalWrite(RELAY_POWER, LOW);
// }

void step_up_pwm(){
  pwm_duty_cycle += pwm_step;
  if( pwm_duty_cycle >PWM_MAX){
    pwm_duty_cycle = PWM_MAX;
  }
  mainled.set(MAP_100_2_255(pwm_duty_cycle));
      // mainled.set(pwm_duty_cycle);
  // analogWrite(PWM_PIN, pwm_duty_cycle);
  Serial.print("step up-- ");
  Serial.println(MAP_100_2_255(pwm_duty_cycle));
}

void step_down_pwm(){
  pwm_duty_cycle -= pwm_step;
  if( pwm_duty_cycle < PWM_MIN){
    pwm_duty_cycle = PWM_MIN;
  }
  mainled.set(MAP_100_2_255(pwm_duty_cycle));
    // mainled.set(pwm_duty_cycle);
  // analogWrite(PWM_PIN, pwm_duty_cycle);
  Serial.print("step dowm-- ");
  Serial.println(MAP_100_2_255(pwm_duty_cycle));
}

// void ticker_cb(){
//     digitalWrite(RELAY_LED, HIGH);  
//     power_state = false;
//     analogWrite(PWM_PIN, 0);
//     digitalWrite(RELAY_POWER, LOW);
//     Serial.print('ticker!');
// }

// void dht_ticker_cb(){
//   float humidity = dht.getHumidity();
//   float temperature = dht.getTemperature();
//   Serial.print(humidity, 1);
//   Serial.print("\t\t");
//   Serial.print(temperature, 1);
//   Serial.print("\t\t");
// }