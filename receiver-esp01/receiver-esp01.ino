//对应接收到的sender的gpio引脚号
#define BUTTON_NUMBER_1 13  // 12 is the number sent by esp-now sender, representing BUTTON_NUMBER_1
#define BUTTON_NUMBER_2 12 // 14 is the number sent by esp-now sender, representing BUTTON_NUMBER_2
#define BUTTON_NUMBER_3 14 // 13 is the number sent by esp-now sender, representing BUTTON_NUMBER_3

//本地io使用情况
#define INPUT_BUTTON_PIN 14  // local control button
#define RELAY_POWER 0   // pin to control relay/power
// #define RELAY_LED 16  // pin to control indicator of power (a led)
// #define PWM_PIN 15  // pin to control fan speed
#define DHT_PIN 2  // pin to control fan speed
// #define PWM_MIN 0
// #define PWM_MAX 100
#include <ESP8266WiFi.h>
#include <espnow.h>
// #include "fastled_effects.h"
// #include "Button2.h"
#include "TickTwo.h"
#include "DHTesp.h"
DHTesp dht;
// Button2 button = Button2(INPUT_BUTTON_PIN);
// Structure example to receive data
// Must match the sender structureshort_timer
typedef struct struct_message {
  char mac[32];
  int buttonNumber;
  float batteryVoltage;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// bool power_state = false;
// int pwm_duty_cycle = PWM_MIN;   //default duty cycle is 40%
// int pwm_step = 20;

void ticker_cb();
// void dht_ticker_cb();

TickTwo short_timer(ticker_cb, 600000, 1);   // 10min delay ,1 time timer
TickTwo long_timer(ticker_cb, 3600000, 1);   // 1h delay ,1 time timer
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
    step_up_pwm();  //第二个按钮，对应长延时关闭继电器
  }
  if (myData.buttonNumber == BUTTON_NUMBER_3){
    step_down_pwm();  //第三个按钮，对应自动
  }
}
 
void setup() {

  pinMode(RELAY_POWER, OUTPUT);
  // pinMode(RELAY_LED, OUTPUT);
  // pinMode(PWM_PIN, OUTPUT);
  analogWriteRange(100);
  digitalWrite(RELAY_POWER, HIGH);  //esp01 默认1为关闭
  // digitalWrite(RELAY_LED, LOW);

  dht.setup(DHT_PIN, DHTesp::DHT11); // Connect DHT sensor to GPIO5  D1
  // button.setClickHandler(single_click_handler);
  // button.setLongClickHandler(longClick_handler);
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  delay(5000);
  WiFi.mode(WIFI_STA);
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

  // dht_timer.start();
}


void loop() {
  // button.loop();
  short_timer.update();
  // dht_timer.update();
  long_timer.update();
  // buzzer_loop();
}


void change_power_state(){
// if( digitalRead(RELAY_POWER) == HIGH ){
//     digitalWrite(RELAY_LED, LOW);  //turn off ws2815 if soundbar is off
//     power_state = false;
//     analogWrite(PWM_PIN, 0);
//   }
//   else{
//     digitalWrite(RELAY_LED, HIGH);  //turn on ws2815 if soundbar is on
//     power_state = true;
//     analogWrite(PWM_PIN, pwm_duty_cycle);
//   }
  digitalWrite(RELAY_POWER, !digitalRead(RELAY_POWER)); // toggle soundbar
}

// void single_click_handler(Button2& btn){ //短按通风10分钟
//   short_timer.start();
// }

// void longClick_handler(Button2& btn){    // 长按关闭
//     digitalWrite(RELAY_LED, LOW);  
//     power_state = false;
//     analogWrite(PWM_PIN, 0);
//     digitalWrite(RELAY_POWER, LOW);
// }

void step_up_pwm(){
  // pwm_duty_cycle += pwm_step;
  // if( pwm_duty_cycle >PWM_MAX){
  //   pwm_duty_cycle = PWM_MAX;
  // }
  // analogWrite(PWM_PIN, pwm_duty_cycle);
  // Serial.print("step up-- ");
  // Serial.println(pwm_duty_cycle);
  digitalWrite(RELAY_POWER, LOW);
  short_timer.start();
  long_timer.stop();
}

void step_down_pwm(){
  // pwm_duty_cycle -= pwm_step;
  // if( pwm_duty_cycle < PWM_MIN){
  //   pwm_duty_cycle = PWM_MIN;
  // }
  // analogWrite(PWM_PIN, pwm_duty_cycle);
  // Serial.print("step dowm-- ");
  // Serial.println(pwm_duty_cycle);
  digitalWrite(RELAY_POWER, LOW);
  short_timer.stop();
  long_timer.start();
}

void ticker_cb(){
    // digitalWrite(RELAY_LED, HIGH);  
    // power_state = false;
    // analogWrite(PWM_PIN, 0);
    digitalWrite(RELAY_POWER, HIGH);  //关闭esp01继电器
    // Serial.print('ticker!');
}

// void dht_ticker_cb(){
//   float humidity = dht.getHumidity();
//   float temperature = dht.getTemperature();
//   Serial.print(humidity, 1);
//   Serial.print("\t\t");
//   Serial.print(temperature, 1);
//   Serial.print("\t\t");
// }