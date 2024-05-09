#include <Servo.h>
#include <Pixy2.h>
#include <R4Timer.h>

Pixy2 pixy;
Servo myServo;

#define echo A2
#define trig A3

//시리얼 수신용 변수
bool rcv_ready = 0;
byte rcv_data = 0;
byte rcv_checksum = 0;
byte rx_buffer[7];
byte rx_data[7];
int rcv_count = 0;
int rcv_index = 0;

//시리얼 송신용 변수
byte tx_data[7];
byte checksum = 0;

//라인트레이싱 용 변수
int num;
int8_t x0;

bool received_check = 0;

int spd = 30;                            // 속도 변수
int spd_turn = 30;                            // 회전 속도 변수 (회전시 오차를 줄이기 위한 속도 감소)
int st_deg_left = 79, st_deg_right = 75;                              // 90도 기준 회전 각도 (오차로 인해 90보다 작음)

int seq = 0, sub_seq = 0;                     // 메인 시퀀스와 서브 시퀀스
bool initial = 1, fin_flag = 0;               // 시퀀스의 처음과 마지막 확인용 변수

int S = 0,                  S_temp = 0;       // 이동 거리 (수신값) , 이동거리 임시저장
int theta_deg = 0,      theta_deg_temp = 0;   // 회전 각도 (수신값), 회전각도 임시저장

bool t2_flag = false;                         // 스케줄링용

//주차장 지정 변수
int room;

// 7-세그먼트용 변수
int p[] = {2, 3, 4, 5, 6, 7, 8, 9};
int D[11][8] {
  {0, 0, 0, 0, 0, 0, 1, 1},
  {1, 0, 0, 1, 1, 1, 1, 1},
  {0, 0, 1, 0, 0, 1, 0, 1},
  {0, 0, 0, 1, 0, 1, 0, 1},
  {1, 0, 0, 1, 1, 0, 0, 1},
  {0, 1, 0, 1, 0, 0, 0, 1},
  {0, 1, 0, 0, 0, 0, 0, 1},
  {0, 0, 0, 1, 1, 1, 1, 1},
  {0, 0, 0, 0, 0, 0, 0, 1},
  {0, 0, 0, 0, 1, 0, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 0},
};

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  for (int i = 0; i < 10; i++) { // 7-세그먼트 2~8번까지 연결
    pinMode(p[i], OUTPUT);
  }
  pinMode(A0, INPUT); // 가변저항 / 왼쪽 GND 가운데 A0 오른쪽 VCC
  pinMode(10, INPUT_PULLUP); // 택트스위치 / 나머지쪽은 GND에 연결

  //송신 데이터의 헤더부분
  tx_data[0] = 0xFF;
  tx_data[1] = 0xFF;

  //픽시 준비
  pixy.init();
  pixy.changeProg("line");

  // 서보 준비 및 초기위치로 설정
  myServo.attach(A5);
  myServo.write(15);
  delay(100);
  myServo.write(20);

  // 가변저항으로 돌려 room 지정 후 택트 스위치를 누르면 출발
  while (digitalRead(10)) {
    room = (analogRead(A0) / 171) + 1;
    for (int i = 0; i < 8; i++) {
      digitalWrite(p[i], D[room][i]);
    }
  }
  delay(500); // 버튼 누르고 0.5초뒤 출발

  // 50ms로 타이머 설정
  Timer1.initialize(50);
  Timer1.attachInterrupt(T2ISR);
  Timer1.start();
}

void loop() {

  // 수신 함수
  serialEvent();

  // 수신 후 데이터 가공
  if (received_check) {
    received_check = 0;
    memcpy((char*)&S, (char*)rx_data, 2);
    memcpy((char*)&theta_deg, (char*)&rx_data[2], 2);
    if (S < 0) S += 65535;
    if (!rx_data[4]) S *= (-1);
    if (theta_deg < 0) theta_deg += 65535;
    if (!rx_data[5]) theta_deg *= (-1);
  }

  // 50ms마다 실행됨

  if (t2_flag) {
    t2_flag = false;
    if (initial) {
      initial = 0;
      theta_deg_temp = theta_deg;
      S_temp = S;
    }
    // 구간별 이동
    switch (seq) {

      case 0:
        spd_scale(30, 50);
         if  (S - S_temp >= 26) spd = spd - (5 * (S - S_temp - 25)); // 238cm 지점부터 5씩 낮춰 250cm지점에서 30에 도달
        foward(30);
        break;

      case 1:
        right(0);
        break;

      case 2:
        spd = 40;
        if (ultrasonic()) {
          foward(60);
        } else {
          sendMotor(0, 0);
        }
        break;

      case 3:
        spd_scale(30, 90);
        if  (S  >= 238) spd = spd - (5 * (S - 237)); // 238cm 지점부터 5씩 낮춰 250cm지점에서 30에 도달
        pixy_linetrace_adaptive();
        if (S >= 250) {
          sendMotor(0, 0);
          initial = 1;
          fin_flag = 1;
        }
        break;

      case 4:
        rightback();
        break;

      case 5:
        moveServo(20, 120);
        initial = 1;
        fin_flag = 1;
        break;

      case 6:
        left(1);
        break;

      case 7:
        spd_scale(30, 40);
        pixy_linetrace_adaptive();
        if (num > 2) {// 선이 1개 초과 검출되면 속도 0을 보냄
          sendMotor(0, 0);
          initial = 1;
          fin_flag = 1;
        }
        break;

      case 8:
        spd_scale(40, 90);
        foward(30);
        break;

      case 9:
        spd_scale(90, 90);
        if  (S - S_temp >= 33) spd = spd - (5 * (S - S_temp - 32)); // 33cm 지점부터 5씩 낮춰 45cm지점에서 30에 도달
        pixy_linetrace_adaptive();
        //5공도착정지

        if  (S - S_temp >= 45) {
          sendMotor(0, 0);
          initial = 1;
          fin_flag = 1;
        }
        break;
      case 10:
        sendMotor(0, 0);
        delay(12000);
        initial = 1;
        fin_flag = 1;
        break;

      case 11:
        spd_scale(30, 90);
        if  (S - S_temp >= 129) spd = spd - (5 * (S - S_temp - 128)); // 127cm 지점부터 5씩 낮춰 139cm지점에서 30에 도달
        pixy_linetrace_adaptive();
        //5공도착정지
        if  (S - S_temp >= 141) {
          initial = 1;
          fin_flag = 1;
        }
        break;

      case 12:
        spd = 30 - (5 * (S - S_temp)); // 0cm 지점부터 5씩 낮춰 4cm지점에서 10에 도달
        if (spd < 13)spd = 13;
        pixy_linetrace(1);           //속도가 낮으므로 더 정확한 라인트레이싱을 할 수 있도록 일반 라인트레이싱 함수 활용
        if  (S - S_temp >= 14) {
          sendMotor(0, 0);
          initial = 1;
          fin_flag = 1;
        }
        break;

      case 13:
        left(0);
        break;

      case 14:
        parking();
        break;

      default:
        sendMotor(0, 0);
        break;
    }
    // 케이스가 마무리 되면 다음 시퀀스로 넘어가도록
    if (fin_flag) {
      fin_flag = 0;
      seq++;
    }
  }
}

// 50ms 마다 t2_flag true로 변환
void T2ISR() {
  t2_flag = true;
}

// 모터 속도 송신 함수
void sendMotor(int m1Spd, int m2Spd) {
  checksum = 0;
  if (m1Spd < 0) {
    tx_data[2] = 0x00; //ccw
    m1Spd *= -1;
  }
  else {
    tx_data[2] = 0x01; //cw
  }
  if (m1Spd > 255) m1Spd = 255;
  tx_data[3] = m1Spd;
  if (m2Spd < 0) {
    tx_data[4] = 0x00; //ccw
    m2Spd *= -1;
  }
  else {
    tx_data[4] = 0x01; //cw
  }
  if (m2Spd > 255) m2Spd = 255;
  tx_data[5] = m2Spd;

  for (int i = 2; i < 6; i++) checksum ^= tx_data[i];
  checksum += 1;
  tx_data[6] = checksum;
  Serial1.write(tx_data, 7);
}

// 속도 조정 / 1cm당 5씩 가속 (속도가 급격히 변하는 것을 방지)
void spd_scale(int start, int limit) {
  spd = start + (5 * (S - S_temp));
  if (spd > limit)spd = limit;
}


/////////////////////////////////////// 픽시 이용 라인트레이싱 함수 ///////////////////////////////////////////
void pixy_linetrace(int line_st) {
  // 모든 라인 특징 받아오기
  pixy.line.getAllFeatures();
  num = pixy.line.numVectors;
  x0 = pixy.line.vectors[0].m_x0;
  int spd_low = spd * 0.3;
  //라인벡터의 x좌표가 가운데 좌표 40보다 왼쪽인 경우
  if (x0 <= 40 - line_st) {
    sendMotor(spd_low, spd);
  }
  //라인벡터의 x좌표가 가운데 좌표 40보다 오른쪽인 경우
  else if (x0 >= 40 + line_st) {
    sendMotor(spd, spd_low);
  } else {
    sendMotor(spd, spd);
  }
}

// 로그함수 그래프를 이용해 선이 중심에서 더 멀어질수록 속도가 급격히 감소하도록 프로그래밍
void pixy_linetrace_adaptive() {
  // 모든 라인 특징 받아오기
  pixy.line.getAllFeatures();
  num = pixy.line.numVectors;
  x0 = pixy.line.vectors[0].m_x0;
  int spd_low;
  //라인벡터의 x좌표가 가운데 좌표 40보다 왼쪽인 경우
  if (x0 < 40) {
    spd_low = spd * (log(x0 - 25) / log(15));
    if (spd_low < 0)spd_low = 0;
    sendMotor(spd_low, spd);
  }
  //라인벡터의 x좌표가 가운데 좌표 40보다 오른쪽인 경우
  else if (x0 > 40) {
    spd_low = spd * (log(55 - x0) / log(15));
    if (spd_low < 0)spd_low = 0;
    sendMotor(spd, spd_low);
  } else {
    sendMotor(spd, spd);
  }
}

//시리얼 수신
void serialEvent() {
  if (Serial1.available()) {
    rcv_data = Serial1.read();
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
        if (rcv_index > 6) {
          rcv_checksum = 0;
          for (int i = 0; i < 6; i++) {
            rcv_checksum ^= rx_buffer[i];
          }
          rcv_checksum += 1;
          if (rcv_checksum == rx_buffer[rcv_index - 1]) {
            memcpy((char*)rx_data, (char*)rx_buffer, 7);
            received_check = 1;
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

  }
}

////////////////////////// 입력한 거리만큼 전진하는 함수 / 거리는 양수만 //////////////////////
void foward(float distance) {
  if (S < S_temp + distance) {
    sendMotor(spd, spd);
  }
  else {
    fin_flag = 1;
    initial = 1;
  }
}

///////////////// 직각 회전 함수 (1이면 외발턴 0이면 제자리턴) //////////////////////////////
void left(bool stat) {
  if (stat) {
    if (theta_deg > theta_deg_temp - st_deg_left) {
      sendMotor(0, spd_turn);
    } else {
      sendMotor(0, 0);
      fin_flag = 1;
      initial = 1;
    }
  }
  else {
    if (theta_deg > theta_deg_temp - st_deg_left) {
      sendMotor(-spd_turn, spd_turn);
    } else {
      sendMotor(0, 0);
      fin_flag = 1;
      initial = 1;
    }
  }
}
void right(bool stat) {
  if (stat) {
    if (theta_deg <  theta_deg_temp + st_deg_right) {
      sendMotor(spd_turn, 0);
    } else {
      sendMotor(0, 0);
      fin_flag = 1;
      initial = 1;
    }
  }
  else {
    if (theta_deg <  theta_deg_temp + st_deg_right) {
      sendMotor(spd_turn, -spd_turn);
    } else {
      sendMotor(0, 0);
      fin_flag = 1;
      initial = 1;
    }
  }
}

// 물건 내려놓기 위한 후진 외발턴 함수
void rightback() {
  if (theta_deg <  theta_deg_temp + 85) {
    sendMotor(0, -spd_turn);
  } else {
    sendMotor(0, 0);
    fin_flag = 1;
    initial = 1;
  }
}

// 서보모터 구동 함수
void moveServo(int startAngle, int endAngle) {
  for (int angle = startAngle; angle <= endAngle; angle += 5) {
    myServo.write(angle);
    delay(10);
  }
}

///////////////////////////////////////초음파 인식///////////////////////////
bool ultrasonic() {
  digitalWrite(trig, LOW);
  digitalWrite(echo, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long duration = pulseIn(echo, HIGH, 4000); // timeout 4ms로 설정 최대 약 33cm까지 감지
  double dist = (double)340 * duration / 20000;
  if (dist < 7 && dist > 1) {
    return 0;
  }
  return 1;
}

// 주차 구역별 주차 함수
void parking() {
  spd = 30;
  switch (room - 1) {
    case 0:
      switch (sub_seq) {
        case 0:
          foward(32);
          break;
        case 1:
          left(0);
          break;
        case 2:
          foward(54);
          break;
        case 3:
          if (theta_deg <  theta_deg_temp + 40) {
            sendMotor(spd_turn, -spd_turn);
          } else {
            sendMotor(0, 0);
            fin_flag = 1;
            initial = 1;
          }
          break;
        default:
          sendMotor(0, 0);
          break;
      }
      break;
    case 1:
      switch (sub_seq) {
        case 0:
          foward(61);
          break;
        case 1:
          left(0);
          break;
        case 2:
          foward(55);
          break;
        case 3:
          if (theta_deg <  theta_deg_temp + 40) {
            sendMotor(spd_turn, -spd_turn);
          } else {
            sendMotor(0, 0);
            fin_flag = 1;
            initial = 1;
          }
          break;
        default:
          sendMotor(0, 0);
          break;
      }
      break;
    case 2:
      switch (sub_seq) {
        case 0:
          foward(94);
          break;
        case 1:
          left(0);
          break;
        case 2:
          foward(55);
          break;
        case 3:
          if (theta_deg <  theta_deg_temp + 40) {
            sendMotor(spd_turn, -spd_turn);
          } else {
            sendMotor(0, 0);
            fin_flag = 1;
            initial = 1;
          }
          break;
        default:
          sendMotor(0, 0);
          break;
      }
      break;
    case 3:
      switch (sub_seq) {
        case 0:
          foward(33);
          break;
        case 1:
          right(0);
          break;
        case 2:
          foward(23);
          break;
        case 3:
          if (theta_deg <  theta_deg_temp + 40) {
            sendMotor(spd_turn, -spd_turn);
          } else {
            sendMotor(0, 0);
            fin_flag = 1;
            initial = 1;
          }
          break;
        default:
          sendMotor(0, 0);
          break;
      }
      break;
    case 4:
      switch (sub_seq) {
        case 0:
          foward(63);
          break;
        case 1:
          right(0);
          break;
        case 2:
          foward(22);
          break;
        case 3:
          if (theta_deg <  theta_deg_temp + 40) {
            sendMotor(spd_turn, -spd_turn);
          } else {
            sendMotor(0, 0);
            fin_flag = 1;
            initial = 1;
          }
          break;
        default:
          sendMotor(0, 0);
          break;
      }
      break;
    case 5:
      switch (sub_seq) {
        case 0:
          foward(94);
          break;
        case 1:
          right(0);
          break;
        case 2:
          foward(23);
          break;
        case 3:
          if (theta_deg <  theta_deg_temp + 40) {
            sendMotor(spd_turn, -spd_turn);
          } else {
            sendMotor(0, 0);
            fin_flag = 1;
            initial = 1;
          }
          break;
        default:
          sendMotor(0, 0);
          break;
      }
      break;
    default:
      sendMotor(0, 0);
      break;
  }

  if (fin_flag) {
    fin_flag = 0;
    sub_seq++;
  }
}
