#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define MOTOR1_A 6
#define MOTOR1_B 5
#define MOTOR2_A 9
#define MOTOR2_B 10

#define SPEED1 3
#define SPEED2 2

#define BRAKE1 A3
#define BRAKE2 A2

#define COLOR1 A0
#define COLOR2 A1

#define LED 4

#define CSN 8
#define CE 7

#define WHEEL_DIAMETER 65
#define HOLES_AMOUNT 42.0
#define PI 3.14

RF24 radio(CE, CSN);
const byte addresses [][6] = {"00001", "00002"};

struct Controll_data{
  bool enable = 0;
  int speed = 0;
};

Controll_data controll_data;

class Motor{
  public:

  int A_PIN, B_PIN;

  int target_speed = 0;
  int actual_speed = 0;
  unsigned long long int smooth_start_timer = 0;

  unsigned long long int encoder_tics = 0, encoder_tics_last = 0;
  unsigned long long int speed_calculation_timer = 0, PID_timer = 0;

  double encoder_speed = 0;
  int real_speed = 0;

  double motor_A, motor_B, kP, kD, kI;

  double eP, eD, eI, last_error = 0, error_rate = 0, error_sum = 0, PID_result, slope_result, error;

  Motor(int a_pin, int b_pin, double motor_a, double motor_b, double kp, double kd, double ki){
    A_PIN = a_pin;
    B_PIN = b_pin;
    motor_A = motor_a;
    motor_B = motor_b;
    kP = kp;
    kD = kd;
    kI = ki;
  }

  void set_actual_speed(){
    if (actual_speed == 0){
      digitalWrite(A_PIN, 0);
      digitalWrite(B_PIN, 0);
    }
    else{
      analogWrite(A_PIN, 255 - actual_speed);
      digitalWrite(B_PIN, 1);
    }
  }

  void speed_tick(){
    if(target_speed < actual_speed){
      actual_speed = target_speed;
      return;
    }

    if (millis() - smooth_start_timer > 10){
      if (target_speed > actual_speed+10){
        actual_speed += 10;
      }
      else{
        actual_speed = target_speed;
      }

      smooth_start_timer = millis();
    }

    set_actual_speed();
  }

  void calculate_speed(){
    if(millis() - speed_calculation_timer > 100){
      encoder_speed = (((double((encoder_tics - encoder_tics_last))/HOLES_AMOUNT)*WHEEL_DIAMETER*PI)*1000)/(millis() - speed_calculation_timer);
      //Serial.println((double((encoder_tics - encoder_tics_last))/HOLES_AMOUNT)*WHEEL_DIAMETER*PI);
      //Serial.println(encoder_speed);
      encoder_tics_last = encoder_tics;
      speed_calculation_timer = millis();
    }
  }

  void PID(){
    if (millis() - PID_timer > 100){
      if(real_speed == 0){
        target_speed = 0;
        return;
      }

      error = (real_speed - encoder_speed);
      eP = error*kP;
      error_rate = error-last_error;
      eD = error_rate*kD;
      last_error = error;
      error_sum += error;
      if (error_sum > 10000){
        error_sum = 10000;
      }
      if (error_sum < -10000){
        error_sum = -10000;
      }
      eI = error_sum * kI;
      PID_result = eP + eD + eI;
      slope_result = (abs(real_speed)*motor_A + motor_B);
      target_speed = int(PID_result + slope_result + 0.5);

      if (target_speed > 255){
        target_speed = 255;
      }
      if (target_speed < 0){
        target_speed = 0;
      }
    PID_timer = millis();
    }
  }

  void tick(){
    calculate_speed();
    PID();
    speed_tick();
  }

};

Motor motor_1(MOTOR1_A, MOTOR1_B, 0.54, -20, 0.3, 0.05, 0.1);
Motor motor_2(MOTOR2_A, MOTOR2_B, 0.54, -20, 0.3, 0.05, 0.1);

void speed1_interrupt(){
  motor_1.encoder_tics ++;
}

void speed2_interrupt(){
  motor_2.encoder_tics ++;
}

void setup() {
  Serial.begin(9600);

  radio.begin();
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();

  pinMode(LED, OUTPUT);

  pinMode(MOTOR1_A, OUTPUT);
  pinMode(MOTOR1_B, OUTPUT);
  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(SPEED1), speed1_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SPEED2), speed2_interrupt, CHANGE);

  motor_1.target_speed = 0;
  motor_2.target_speed = 0;
}

void loop() {
  if (radio.available()) {
    radio.read(&controll_data, sizeof(controll_data));

    if (controll_data.enable){
      digitalWrite(LED, 1);

      motor_1.real_speed = controll_data.speed;
      motor_2.real_speed = controll_data.speed;
    }
    else{
      digitalWrite(LED, 0);

      motor_1.real_speed = 0;
      motor_2.real_speed = 0;
    }
  }

  if(Serial.available()){
    char data = Serial.read();
    if(data == 'P'){
      motor_2.kP = Serial.parseFloat();
    }
    if(data == 'D'){
      motor_2.kD = Serial.parseFloat();
    }
    if(data == 'I'){
      motor_2.kI = Serial.parseFloat();
    }
    if(data == 'S'){
      motor_2.real_speed = Serial.parseInt();
    }
  }

  motor_1.tick();
  motor_2.tick();

  /*Serial.print(motor_1.encoder_speed);
  Serial.print(" ");
  Serial.println(motor_2.encoder_speed);*/
  Serial.print(int(motor_2.encoder_speed));
  Serial.print(" ");
  Serial.print(int(motor_2.real_speed));
  Serial.print(" ");
  Serial.print(int(motor_2.target_speed));
  Serial.print(" ");
  Serial.print(int(motor_2.actual_speed));
  Serial.print(" ");
  Serial.println(int(motor_2.error_sum));
  //Serial.println(int(motor_2.encoder_speed));
  delay(10);
}