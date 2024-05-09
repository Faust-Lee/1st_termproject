#include "BlackpillTimer.h"

#define ENC1_CHA  PB6
#define ENC1_CHB  PB7
#define M1_PWM    PB0
#define M1_DIR1   PB12
#define M1_DIR2   PB13

#define ENC2_CHA  PB8
#define ENC2_CHB  PB9
#define M2_PWM    PB1
#define M2_DIR1   PB14
#define M2_DIR2   PB15

#define STBY      PA8


BlackpillTimer Timer(TIM1); // TIM1 사용

HardwareSerial Serial2(PA3, PA2);

int m1Pos = 0,            m2Pos = 0;
int m1Spd = 0,            m2Spd = 0;
int m1Angle = 0,        m2Angle = 0;

float Kp1 = 0.5,          Kp2 = 0.4;
float Ki1 = 0.5,          Ki2 = 0.5;
float Kd1 = 0.3,          Kd2 = 0.05;

float m1speed = 0,        m2speed = 0;
float m1_ref_spd = 0,     m2_ref_spd = 0;
float m1_err_spd = 0,     m2_err_spd = 0;
float m1_err_spd_k_1 = 0, m2_err_spd_k_1 = 0;
float m1_derr_spd = 0,    m2_derr_spd = 0;
float m1_err_sum = 0,     m2_err_sum = 0;

float ctrl_up1 = 0,       ctrl_up2 = 0;
float ctrl_ui1 = 0,       ctrl_ui2 = 0;
float ctrl_ud1 = 0,       ctrl_ud2 = 0;

int ctrl_u1 = 0,          ctrl_u2 = 0;
int ipwm_u1 = 0,          ipwm_u2 = 0;

long e1cnt = 0,            e2cnt = 0;
long e1cnt_k = 0, e1cnt_k_1 = 0, d_e1cnt = 0;
long e2cnt_k = 0, e2cnt_k_1 = 0, d_e2cnt = 0;

bool t2_flag = false;

bool rcv_status = 0;
bool rcv_ready = 0;
byte rcv_data = 0;
byte rcv_checksum = 0;
byte rx_buffer[6];
byte rx_data[6];
int rcv_count = 0;
int rcv_index = 0;

//for motor UART
byte tx_data[9];
byte checksum = 0;

//차체 크기 측정값
float L = 19.00;
float Wheel_R = 3.14;
int S = 0;
int theta_deg = 0;

void setup() {
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(STBY, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), m1chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_CHB), m1chB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), m2chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHB), m2chB_ISR, CHANGE);

  Timer.setIntervalMs(50);
  Timer.attachUpdateInterrupt(t2_ISR);  // 1000 us = 1 ms
  Timer.start();

  // Header
  Serial2.begin(115200);
  tx_data[0] = 0xFF;
  tx_data[1] = 0xFF;

  digitalWrite(STBY, HIGH);
}

void loop() {
  if (Serial2.available()) {
    rcv_data = Serial2.read();
    switch (rcv_count) {
      case 0:
        if ((rcv_ready == 0) && (rcv_data == 0xFF)) {
          rcv_count = 1;
        }
        else
          rcv_count = 0;
        break;
      case 1:
        if ((rcv_ready == 0) && (rcv_data == 0xFF)) {
          rcv_count = 2;
          rcv_ready = 1;
        }
        else
          rcv_count = 0;
        break;
      case 2:
        rx_buffer[rcv_index] = rcv_data;
        rcv_index++;
        if (rcv_index > 4) {
          rcv_checksum = 0;
          for (int i = 0; i < 4; i++) {
            rcv_checksum ^= rx_buffer[i];
          }
          rcv_checksum += 1;
          if (rcv_checksum == rx_buffer[rcv_index - 1]) {
            rcv_status = 1;
            memcpy((char*)rx_data, (char*)rx_buffer, 5);
          }
          rcv_count = 0;
          rcv_index = 0;
          rcv_ready = 0;
        }
        break;
      default:
        rcv_count = 0;
        rcv_index = 0;
        rcv_ready = 0;
        break;
    }

    if (rx_data[0] == 0x00) m1_ref_spd = rx_data[1] * (-1);
    else                   m1_ref_spd = rx_data[1];
    if (rx_data[2] == 0x00) m2_ref_spd = rx_data[3] * (-1);
    else                   m2_ref_spd = rx_data[3];
  }




  if (t2_flag) {
    t2_flag = 0;

    m1speed = d_e1cnt * 10 / 11;
    m2speed = -d_e2cnt * 10 / 11;

    // ERROR
    m1_err_spd = m1_ref_spd - m1speed;
    m1_derr_spd = m1_err_spd - m1_err_spd_k_1;
    m1_err_sum = m1_err_sum + m1_err_spd;
    m1_err_spd_k_1 = m1_err_spd;

    m2_err_spd = m2_ref_spd - m2speed;
    m2_derr_spd = m2_err_spd - m2_err_spd_k_1;
    m2_err_sum = m2_err_sum + m2_err_spd;
    m2_err_spd_k_1 = m2_err_spd;

    //PID
    ctrl_up1 = Kp1 * m1_err_spd;
    ctrl_ui1 = Ki1 * m1_err_sum;
    ctrl_ud1 = Kd1 * m1_derr_spd;
    ctrl_up2 = Kp2 * m2_err_spd;
    ctrl_ui2 = Ki2 * m2_err_sum;
    ctrl_ud2 = Kd2 * m2_derr_spd;

    ctrl_u1 = (int)(ctrl_up1 + ctrl_ui1 + ctrl_ud1);
    ctrl_u2 = (int)(ctrl_up2 + ctrl_ui2 + ctrl_ud2);

    if (ctrl_u1 > 255)       ctrl_u1 = 255;
    else if (ctrl_u1 < -255) ctrl_u1 = -255;
    if (ctrl_u2 > 255)       ctrl_u2 = 255;
    else if (ctrl_u2 < -255) ctrl_u2 = -255;

    motor1_drive(ctrl_u1);
    motor2_drive(ctrl_u2);

    m1Angle = (int)e1cnt / 11.0 * 3.0;
    m2Angle = (int)e2cnt / 11.0 * 3.0;


    float theta_1 = m1Angle * PI / 180; //각도를 degree값으로 변환
    float theta_2 = -m2Angle * PI / 180;
    // Odometry Parameter
    float S_R = Wheel_R * theta_1;
    float S_L = Wheel_R * theta_2;
    S = (S_R + S_L) / 2;

    float theta = (S_R - S_L) / L;
    theta_deg = theta * 180 / PI;



    sendOdometry(S, theta_deg);
  }
}

void sendOdometry(int s, int theta) {
  checksum = 0;
  if (s < 0) {
    tx_data[6] = 0x00; //ccw
    s *= -1;
  }
  else tx_data[6] = 0x01; //cw
  //  if(s>65535) s=0;
  memcpy(&tx_data[2], &s, 2);

  if (theta < 0) {
    tx_data[7] = 0x00; //ccw
    theta *= -1;
  }
  else tx_data[7] = 0x01; //cw
  //  if(m2>65535) m2=0;
  memcpy(&tx_data[4], &theta, 2);

  for (int i = 2; i < 8; i++) checksum ^= tx_data[i];
  checksum += 1;
  tx_data[8] = checksum;
  Serial2.write(tx_data, 9);
}

// Timer
void t2_ISR() {
  t2_flag = true;
  e1cnt_k = e1cnt;
  e2cnt_k = e2cnt;
  d_e1cnt = e1cnt_k - e1cnt_k_1;
  d_e2cnt = e2cnt_k - e2cnt_k_1;
  e1cnt_k_1 = e1cnt_k;
  e2cnt_k_1 = e2cnt_k;
}

// Motor Control
void motor1_drive(int spd) {
  if (spd >= 0) {
    digitalWrite(M1_DIR1, HIGH);
    digitalWrite(M1_DIR2, LOW);
    digitalWrite(STBY, HIGH);
  }
  else {
    digitalWrite(M1_DIR1, LOW);
    digitalWrite(M1_DIR2, HIGH);
    digitalWrite(STBY, HIGH);
  }
  analogWrite(M1_PWM, abs(spd));
}

void motor2_drive(int spd) {
  if (spd >= 0) {
    digitalWrite(M2_DIR1, HIGH);
    digitalWrite(M2_DIR2, LOW);
    digitalWrite(STBY, HIGH);
  }
  else {
    digitalWrite(M2_DIR1, LOW);
    digitalWrite(M2_DIR2, HIGH);
    digitalWrite(STBY, HIGH);
  }
  analogWrite(M2_PWM, abs(spd));
}


// ENCODER Measure
void m1chA_ISR() {
  if (digitalRead(ENC1_CHA) == HIGH) {
    if (digitalRead(ENC1_CHB) == LOW)  e1cnt--;
    else                              e1cnt++;
  }
  else {
    if (digitalRead(ENC1_CHB) == HIGH) e1cnt--;
    else                              e1cnt++;
  }
}

void m1chB_ISR() {
  if (digitalRead(ENC1_CHB) == HIGH) {
    if (digitalRead(ENC1_CHA) == HIGH)  e1cnt--;
    else                              e1cnt++;
  }
  else {
    if (digitalRead(ENC1_CHA) == LOW) e1cnt--;
    else                              e1cnt++;
  }
}

void m2chA_ISR() {
  if (digitalRead(ENC2_CHA) == HIGH) {
    if (digitalRead(ENC2_CHB) == LOW) e2cnt--;
    else                              e2cnt++;
  }
  else {
    if (digitalRead(ENC2_CHB) == HIGH)  e2cnt--;
    else                              e2cnt++;
  }
}

void m2chB_ISR() {
  if (digitalRead(ENC2_CHB) == HIGH) {
    if (digitalRead(ENC2_CHA) == HIGH) e2cnt--;
    else                              e2cnt++;
  }
  else {
    if (digitalRead(ENC2_CHA) == LOW)  e2cnt--;
    else                              e2cnt++;
  }
}
