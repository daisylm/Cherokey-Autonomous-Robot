/***************************************************
DFRobot
ROB0117 Cherokey 4WD
Sonar Dodge
***************************************************
This example uses a URM sensor to drive the robot and avoid obstacles

Updated 2015-12-31
By Matt

GNU Lesser General Public License.
See <http://www.gnu.org/licenses/> for details.
All above must be included in any redistribution
****************************************************/

/***********Notice and Troubleshooting***************
For help and info visit the wiki page for this product:
https://www.dfrobot.com/wiki/index.php?title=Basic_Kit_for_Cherokey_4WD_SKU:ROB0117
For any other problems, post on the DFRobot forum or email techsupport@dfrobot.com
****************************************************/
#include <Servo.h>
#include <Metro.h>

Metro measureDistance = Metro(50);
Metro sweepServo = Metro(100);
// Metro carActionTimer = Metro(100);  // 用于控制小车动作的计时器

int speedPin_M1 = 5;      //M1 Speed Control
int speedPin_M2 = 6;      //M2 Speed Control
int directionPin_M1 = 4;  //M1 Direction Control
int directionPin_M2 = 7;  //M2 Direction Control
unsigned long actualDistance = 0;

Servo myservo;  // create servo object to control a servo
int pos = 60;
int sweepFlag = 1;
int directionFLag = 0;
unsigned long savedDistance = 0;

int URPWM = 3;                                     // PWM Output 0－25000US，Every 50US represent 1cmk
int URTRIG = 10;                                   // PWM trigger pin
uint8_t EnPwmCmd[4] = { 0x44, 0x02, 0xbb, 0x01 };  // distance measure command

// 状态机变量
enum CarState {
  FORWARD,
  BACKWARD,
  TURN_RIGHT,
  TURN_LEFT,
  SERVO_RESET
};

CarState currentState = FORWARD;
unsigned long stateStartTime = 0;
bool obstacleDetected = false;

void setup() {  // Serial initialization
  myservo.attach(9);
  Serial.begin(9600);  // Sets the baud rate to 9600
  SensorSetup();
  delay(1000);
  carAdvance(150, 150);
  currentState = FORWARD;
  stateStartTime = millis();
}

void loop() {
  // 状态机处理
  handleCarMovement();
}

void handleCarMovement() {
  unsigned long currentTime = millis();

  switch (currentState) {
    case FORWARD:
      // 舵机扫描
      if (sweepServo.check() == 1) {
        servoSweep();
      }
      // 距离测量
      if (measureDistance.check() == 1) {
        actualDistance = MeasureDistance();
        Serial.print(pos);
        Serial.print(": ");
        Serial.println(actualDistance);

        // 检测障碍物
        obstacleDetected = (actualDistance <= 15);
      }
      // servoSweep();
      // actualDistance = MeasureDistance();
      // Serial.print(pos);
      // Serial.print(": ");
      // Serial.println(actualDistance);

      // // 检测障碍物
      // obstacleDetected = (actualDistance <= 15);
      if (obstacleDetected) {
        // 检测到障碍物，开始后退
        carBack(200, 200);
        currentState = BACKWARD;
        stateStartTime = currentTime;
        directionFLag = pos;
        savedDistance = actualDistance;
        Serial.print(pos);
        Serial.print(": ");
        Serial.println(actualDistance);
        Serial.println("State: BACKWARD");
      }
      // 如果没有障碍物，继续前进（已经在setup中设置）
      break;

    case BACKWARD:
      if (currentTime - stateStartTime >= 250) {
        // 后退完成，开始转向
        if (directionFLag >= 91) {
          carTurnRight(155, 155);
          currentState = TURN_RIGHT;
          Serial.print(directionFLag);
          Serial.print(": ");
          Serial.println(savedDistance);
          Serial.println("State: TURN_RIGHT");
        } else {
          carTurnLeft(155, 155);
          currentState = TURN_LEFT;
          Serial.print(directionFLag);
          Serial.print(": ");
          Serial.println(savedDistance);
          Serial.println("State: TURN_LEFT");
        }
        stateStartTime = currentTime;
      }
      break;

    case TURN_RIGHT:
      if (currentTime - stateStartTime >= 400) {
        // 右转完成，重置舵机
        myservo.write(90);
        currentState = SERVO_RESET;
        stateStartTime = currentTime;
      }
      break;

    case TURN_LEFT:
      if (currentTime - stateStartTime >= 400) {
        // 左转完成，重置舵机
        myservo.write(105);
        currentState = SERVO_RESET;
        stateStartTime = currentTime;
      }
      break;

    case SERVO_RESET:
      if (currentTime - stateStartTime >= 1000) {
        // 舵机重置完成，恢复前进
        carAdvance(150, 150);
        currentState = FORWARD;
        stateStartTime = currentTime;
        Serial.print(pos);
        Serial.print(": ");
        Serial.println(actualDistance);
        Serial.println("State: FORWARD");
      }
      break;
  }
}

void SensorSetup() {
  pinMode(URTRIG, OUTPUT);     // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG, HIGH);  // Set to HIGH
  pinMode(URPWM, INPUT);       // Sending Enable PWM mode command
  for (int i = 0; i < 4; i++) {
    Serial.write(EnPwmCmd[i]);
  }
}

int MeasureDistance() {  // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);  // reading Pin PWM will output pulses
  unsigned long distance = pulseIn(URPWM, LOW);
  if (distance / 50 <= 5) {  // the reading is invalid.
    Serial.print("Invalid");
    return actualDistance;
  } else {
    distance = distance / 50;  // every 50us low level stands for 1cm
  }
  return distance;
}

void carStop() {  //  Motor Stop
  digitalWrite(speedPin_M2, 0);
  digitalWrite(directionPin_M1, LOW);
  digitalWrite(speedPin_M1, 0);
  digitalWrite(directionPin_M2, LOW);
}

void carAdvance(int leftSpeed, int rightSpeed) {  //Move backward
  analogWrite(speedPin_M2, leftSpeed);            //PWM Speed Control
  digitalWrite(directionPin_M1, LOW);             //set LOW to reverse or HIGH to advance
  analogWrite(speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}

void carBack(int leftSpeed, int rightSpeed) {  //Move forward
  analogWrite(speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite(speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, HIGH);
}

void carTurnRight(int leftSpeed, int rightSpeed) {  //Turn Left
  analogWrite(speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, LOW);
  analogWrite(speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, HIGH);
}

void carTurnLeft(int leftSpeed, int rightSpeed) {  //Turn Right
  analogWrite(speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite(speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}

void servoSweep() {
  //Serial.println(pos);
  if (sweepFlag) {
    if (pos >= 60 && pos <= 150) {
      pos = pos + 15;      // in steps of 1 degree
      myservo.write(pos);  // tell servo to go to position in variable 'pos'
    }
    if (pos > 149) sweepFlag = false;  // assign the variable again
  } else {
    if (pos >= 60 && pos <= 150) {
      pos = pos - 15;
      myservo.write(pos);
    }
    if (pos < 61) sweepFlag = true;
  }
}