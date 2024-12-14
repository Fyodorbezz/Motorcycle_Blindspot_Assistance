#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define BUTTTON A5
#define LED1 5
#define LED2 2

#define CE 7
#define CSN 8

RF24 radio(CE, CSN);
const byte addresses [][6] = {"00001", "00002"};

bool prev_button_state = 0;
unsigned long long debounce_timer = 0;

bool work_state = 0;

struct Controll_data{
  bool enable = 0;
  int speed = 0;
};

Controll_data controll_data;

void setup() {

  Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.stopListening();

  pinMode(BUTTTON, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop() {
  if (digitalRead(BUTTTON) == 0 && prev_button_state == 1 && millis() - debounce_timer > 100){
    debounce_timer = millis();
    prev_button_state = 0;

    if (work_state == 0){
      controll_data.enable = 1;
      controll_data.speed = 550;

      radio.write(&controll_data, sizeof(controll_data));
      digitalWrite(LED1, 1);

      work_state = 1;
    }
    else{
      controll_data.enable = 0;
      controll_data.speed = 0;

      radio.write(&controll_data, sizeof(controll_data));
      digitalWrite(LED1, 0);

      work_state = 0;
    }
  }
  else if(digitalRead(BUTTTON) == 1){
    prev_button_state = 1;
  }

}