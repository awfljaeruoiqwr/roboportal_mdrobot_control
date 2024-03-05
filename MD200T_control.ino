#include <IBusBM.h> // RC 수신기 수신을 위한 라이브러리

IBusBM IBus;    // IBus object

// 시리얼 2개를 사용한 제어 : 성공함
#define MOTOR_CONTROLLER_MACHINE_ID   183
#define USER_MACHINE_ID               184
#define ID                            1
#define PID_PNT_VEL_CMD               207
#define PID_PNT_MAIN_TOTAL_DATA_NUM   24 
#define PID_MAIN_DATA                 193
#define ENABLE                        1  
#define RETURN_PNT_MAIN_DATA          2    
#define MAX_PACKET_SIZE               255
#define DE_RE_PIN1                     3
#define DE_RE_PIN2                     2

//속도를 RPM으로 바꾸는 상수
#define LEFT  0
#define RIGHT 1
#define VELOCITY_CONSTANT_VALUE       9.5492743

int SW1_Pin = 8; // 스위치가 연결된 핀 번호
int SW1_state = 0; // 스위치의 상태를 저장하는 변수
int SW2_Pin = 9; // 스위치가 연결된 핀 번호
int SW2_state = 0; // 스위치의 상태를 저장하는 변수
int E_STOP_Pin = 12; // 스위치가 연결된 핀 번호
int E_STOP_state = 0; // 스위치의 상태를 저장하는 변수

typedef unsigned char  BYTE;
typedef unsigned int   WORD;

typedef struct {
  BYTE byLow;
  BYTE byHigh;
} IByte;

int Byte2Int(BYTE byLow, BYTE byHigh) {
  return (byLow | (int)byHigh << 8);
}

long Byte2Long(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4) {
  return ((long)byData1 | (long)byData2 << 8 | (long)byData3 << 16 | (long)byData4 << 24);
}

IByte Int2Byte(int nIn) {
  IByte Ret;
  Ret.byLow = nIn & 0xff;
  Ret.byHigh = nIn >> 8 & 0xff;
  return Ret;
}

//조이스틱 x축(초록선) 전압값을 속도로 바꾸는 함수
float convertVoltageToSpeed(float voltage_x) {
      if (voltage_x <= 0.4) {
    return 0.0;
  }

  float minCenterVoltage = 2.3;
  float maxCenterVoltage = 2.7;
  float maxSpeed = 0.6; // m/s
  
  if (voltage_x >= minCenterVoltage  && voltage_x <= maxCenterVoltage) {
    return 0.0;
  } else if (voltage_x < minCenterVoltage) {
    return(voltage_x - minCenterVoltage) * (maxSpeed / (4.5 - minCenterVoltage));
  } else {
    return (voltage_x - maxCenterVoltage) * (maxSpeed / (maxCenterVoltage - 0.5));
  }
}

//조이스틱 y축(흰색선)전압값을 각속도로 바꾸는 함수
float convertVoltageToAngularSpeed(float voltage_y) {
    if (voltage_y <= 0.4) {
    return 0.0;
  }

  float minCenterVoltage = 2.3;
  float maxCenterVoltage = 2.7;
  float maxAngularSpeed = 0.5; // rad/s
  
  if (voltage_y >= minCenterVoltage && voltage_y <= maxCenterVoltage) {
    return 0.0;
  } else if (voltage_y < minCenterVoltage) {
    return (voltage_y - minCenterVoltage) * (-maxAngularSpeed / (minCenterVoltage - 0.5));
  } else {
    return (voltage_y - maxCenterVoltage) * (-maxAngularSpeed / (4.5 - maxCenterVoltage));
  }
}

//RC 채널 3번 값을 속도로 바꾸는 함수
float convertInputToSpeed(int input) {
  if (input == 0) {
    return 0.0;
  }

  int centerValue = 1500;
  int minValue = 1000;
  int maxValue = 2000;
  float maxSpeed = 1; // m/s
  
  if (input == centerValue) {
    return 0.0;
  } else if (input < centerValue) {
    return (float)(input - centerValue) * (maxSpeed / (centerValue - minValue));
  } else {
    return (float)(input - centerValue) * (maxSpeed / (maxValue - centerValue));
  }
}
//RC 채널 1번 값을 각속도로 바꾸는 함수
float convertInputToAngularSpeed(int input) {
  if (input == 0) {
    return 0.0;
  }

  int centerValue = 1500;
  int minValue = 1000;
  int maxValue = 2000;
  float maxAngularSpeed = 1; // rad/s
  
  if (input == centerValue) {
    return 0.0;
  } else if (input < centerValue) {
    return (float)(centerValue - input) * (-maxAngularSpeed / (centerValue - minValue));
  } else {
    return (float)(centerValue - input) * (-maxAngularSpeed / (maxValue - centerValue));
  }
}

//속도를 RPM으로 바꾸는 함수
int16_t goal_rpm_speed[2];

void RobotSpeedToRPMSpeed(double linear, double angular) {
    double wheel_velocity_cmd[2];

    double wheel_radius = 0.085;
    double wheel_separation = 0.68;
    double reduction = 1;
    double nMaxRPM =200;

    wheel_velocity_cmd[LEFT]   = linear + (angular * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT]  = linear - (angular * wheel_separation / 2);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT]  * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -nMaxRPM, nMaxRPM);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -nMaxRPM, nMaxRPM);
    
    // 400T에 사용
    // goal_rpm_speed[0] = (int16_t)(wheel_velocity_cmd[LEFT]);
    // goal_rpm_speed[1] = (int16_t)(-wheel_velocity_cmd[RIGHT]); //여기 부호를 바꿔주면 방향이 바뀐다 

    // 200T에 사용
    goal_rpm_speed[0] = (int16_t)(wheel_velocity_cmd[LEFT]);
    goal_rpm_speed[1] = (int16_t)(wheel_velocity_cmd[RIGHT]); //여기 부호를 바꿔주면 방향이 바뀐다 
}

void setup() {
  // 시리얼 통신 설정
  IBus.begin(Serial);    // iBUS object connected to serial0 RX pin
  Serial1.begin(115200);  // Baudrate 설정 Serial1로 변경
  Serial2.begin(115200);
  Serial2.setTimeout(2000); // 2초의 timeout으로 설정합니다.
  pinMode(E_STOP_Pin, INPUT_PULLUP); //E-stop 스위치 핀 풀업 설정
  pinMode(SW1_Pin, INPUT_PULLUP); //스위치 핀 풀업 설정
  pinMode(SW2_Pin, INPUT_PULLUP); //스위치 핀 풀업 설정
  pinMode(DE_RE_PIN1, OUTPUT);
  digitalWrite(DE_RE_PIN1, LOW);  // 처음에는 수신 모드로 설정
  pinMode(DE_RE_PIN2, OUTPUT);
  digitalWrite(DE_RE_PIN2, LOW);
}


float follow_linear_speed = 0;
float follow_angular_speed = 0;
int ready = 0 ;
int tbd = 0;

void loop() {

  // 스위치 입력

  E_STOP_state = digitalRead(E_STOP_Pin);
  SW1_state = digitalRead(SW1_Pin);
  SW2_state = digitalRead(SW2_Pin);

  // A0,A1 핀을 통한 아날로그 값 받기 (0~1023 사이 정수) 조이스틱 입력
  int sensorValue_x = analogRead(A0);
  int sensorValue_y = analogRead(A1);

  // 0~1023 사이 정수 값을 (0~5V의 전압 값으로 다시 변환
  float voltage_x = sensorValue_x * (5.0 / 1023.0);
  float voltage_y = sensorValue_y * (5.0 / 1023.0);
    
  //전압을 속도,각속도로 바꿔서 출력해주는 부분
  float joy_linear_speed = convertVoltageToSpeed(voltage_x);
  float joy_angular_speed = convertVoltageToAngularSpeed(voltage_y);


// RC i-bus 입력
  int val1, val2, val3, val4, val5, val6;
  val1 = IBus.readChannel(0); // channel 1 좌우
  // val2 = IBus.readChannel(1);
  val3 = IBus.readChannel(2); // channel 3 앞뒤
  // val4 = IBus.readChannel(3);
  val5 = IBus.readChannel(4);
  val6 = IBus.readChannel(5);
  
  float RC_linear_speed = convertInputToSpeed(val3);
  float RC_angular_speed = convertInputToAngularSpeed(val1);

  // Serial.available() 함수를 사용하여 데이터가 사용 가능한지 확인
  if (Serial2.available()) {
    char buffer[256]; // 충분한 크기의 버퍼를 선언합니다.
    int length = sizeof(buffer);
    int n = Serial2.readBytesUntil('\n', buffer, length);
    buffer[n] = '\0'; // 문자열의 끝을 나타내는 null 문자를 추가합니다.
    String received = String(buffer);
    int firstSeparatorIndex = received.indexOf(',');
    int secondSeparatorIndex = received.indexOf(',', firstSeparatorIndex + 1);
    int thirdSeparatorIndex = received.indexOf(',', secondSeparatorIndex + 1);
    
    if (firstSeparatorIndex != -1 && secondSeparatorIndex != -1 && thirdSeparatorIndex != -1) {
      String linearStr = received.substring(0, firstSeparatorIndex);
      String angularStr = received.substring(firstSeparatorIndex + 1, secondSeparatorIndex);
      String readyStr = received.substring(secondSeparatorIndex + 1, thirdSeparatorIndex);
      String tbdStr = received.substring(thirdSeparatorIndex + 1);
      
      follow_linear_speed = linearStr.toFloat();
      follow_angular_speed = angularStr.toFloat();
      ready = readyStr.toInt(); // 'ready' 데이터를 정수로 변환합니다.
      tbd = tbdStr.toInt(); // 'sig' 데이터를 정수로 변환합니다.,{}
    }
  }
  else {
    follow_linear_speed = 0;
    follow_angular_speed = 0;
    ready = 0;
    tbd = 0;
  }

  float linear_speed, angular_speed;

    if (SW1_state == 1 & SW2_state == 0) {
      linear_speed = RC_linear_speed;
      angular_speed = RC_angular_speed;
  } else if (SW1_state == 0 & SW2_state == 1) {
      linear_speed = follow_linear_speed;
      angular_speed = follow_angular_speed;
  } else {
      linear_speed = joy_linear_speed;
      angular_speed = joy_angular_speed;
  }

  // m/s,rad/s를 rpm으로 변환해서 left_rpm, right_rpm으로 출력하는 코드
  RobotSpeedToRPMSpeed(linear_speed, angular_speed);



  // 주기적으로 명령 전송
  delay(50);
  // RobotSpeedToRPMSpeed 명령 모터 드라이버로 전송
  if (E_STOP_state == 0) {
    PutPNTVelCmd(goal_rpm_speed[LEFT], goal_rpm_speed[RIGHT]);
  }
  else {
    PutPNTVelCmd(0, 0);
  }  

}

void PutPNTVelCmd(int nLeftRPM, int nRightRPM) {
  BYTE byD[MAX_PACKET_SIZE];
  BYTE byChkSum = 0, byDataNum;
  IByte iData;
  byDataNum = 7;
  
  byD[0] = MOTOR_CONTROLLER_MACHINE_ID;
  byD[1] = USER_MACHINE_ID;
  byD[2] = ID;
  byD[3] = PID_PNT_VEL_CMD;
  byD[4] = byDataNum;
  byD[5] = ENABLE;
  iData = Int2Byte(nLeftRPM);
  byD[6] = iData.byLow;
  byD[7] = iData.byHigh;
  byD[8] = ENABLE;
  iData = Int2Byte(nRightRPM);
  byD[9] = iData.byLow;
  byD[10] = iData.byHigh;
  byD[11] = RETURN_PNT_MAIN_DATA;
  for (int i = 0; i < 12; i++) byChkSum += byD[i];
  byD[12] = ~(byChkSum) + 1;

  // RS-485 전송 모드로 전환
  digitalWrite(DE_RE_PIN1, HIGH);
  delay(10);  // 안정화를 위한 딜레이

  // // 데이터 전송 전 디버그 메시지 출력
  // Serial.println("Serial1로 데이터 전송 시작...");

  // 데이터 전송
  for (int i = 0; i < 13; i++) {
    Serial1.write(byD[i]);
  }

  // // 데이터 전송 후 디버그 메시지 출력
  // Serial.println("Serial1로 데이터 전송 완료.");

  // 전송 완료 후 RS-485 수신 모드로 전환
  delay(10);  // 전송 완료를 위한 딜레이
  digitalWrite(DE_RE_PIN1, LOW);
}


