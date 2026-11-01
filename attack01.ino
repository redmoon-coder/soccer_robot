#include <PeanutKingSoccerV4.h>

// Constants
const int MOTOR_SPEED_SLOW = 60;
const int MOTOR_SPEED_MEDIUM = 90;
const int MOTOR_SPEED_FAST = 120;
const int BALL_CLOSE_THRESHOLD = 200;
const int BALL_VISIBLE_THRESHOLD = 100;
const int ULTRASONIC_CLOSE = 400;
const int ULTRASONIC_FAR = 950;
const int ULTRASONIC_BACK_CLOSE = 300;
const int ULTRASONIC_BACK_FAR = 400;
const int COMPASS_TOLERANCE = 15;
const unsigned long NO_BALL_TIMEOUT = 2000;

// Robot instance
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

// Sensor readings
int angle = 0;
int leftUltrasonic = 0;
int backUltrasonic = 0;
int strongestEye = 0;
int strongestEyeValue = 0;
unsigned long lastBallSeenTime = 0;

enum RobotState {
  STATE_COMPASS_CORRECTION,  // Highest priority state
  STATE_ATTACK,              // Ball is very close
  STATE_GOALKEEPER,
  STATE_SEARCH,
  STATE_PROTECT
};
RobotState currentState = STATE_COMPASS_CORRECTION;

void setup() {
  robot.init();
  Serial.begin(115200);
  lastBallSeenTime = millis();
}

// Motor control functions remain unchanged
void setMotors(int frontRight, int rearRight, int rearLeft, int frontLeft) {
  robot.motorSet(0, frontRight);
  robot.motorSet(1, rearRight);
  robot.motorSet(2, rearLeft);
  robot.motorSet(3, frontLeft);
}

void stopMotors() {
  setMotors(0, 0, 0, 0);
}

// Movement helper functions (same as before)
void attack(int speed) { setMotors(-speed, -speed, speed, speed); }
void seek2(int speed) { setMotors(speed, 0, -speed, 0); }
void seek6(int speed) { setMotors(0, speed, 0, -speed); }
void seek1(int speed) { setMotors(speed, -speed, -speed, speed); }
void seek7(int speed) { setMotors(-speed, speed, speed, -speed); }
void seek011(int speed) { setMotors(0, speed, 0, -speed); }
void seek89(int speed) { setMotors(speed, 0, -speed, 0); }
void seek10(int speed) { setMotors(speed, -speed, -speed, speed); }
void turnRight(int speed) { setMotors(speed, speed, speed, speed); }
void turnLeft(int speed) { setMotors(-speed, -speed, -speed, -speed); }

void updateSensors() {
  angle = robot.compassRead();
  leftUltrasonic = robot.ultrasonicRead(U2);
  backUltrasonic = robot.ultrasonicRead(U1);
  strongestEye = robot.compoundMaxEye();
  strongestEyeValue = robot.compoundMaxEyeVal();
}

void determineState() {
  // Critical ball detection - highest priority action
  if (strongestEyeValue > BALL_CLOSE_THRESHOLD) {
    currentState = STATE_ATTACK;
    return;
  }
  
  // Ball timeout handling
  if (strongestEyeValue < BALL_VISIBLE_THRESHOLD) {
    if (millis() - lastBallSeenTime > NO_BALL_TIMEOUT) {
      currentState = STATE_PROTECT;
      return;
    }
  } else {
    lastBallSeenTime = millis();
  }

  // Compass correction needed (unless attacking)
  if (abs(angle) > COMPASS_TOLERANCE && angle < (360 - COMPASS_TOLERANCE)) {
    currentState = STATE_COMPASS_CORRECTION;
    return;
  }

  // Normal state transitions
  if (strongestEyeValue > BALL_VISIBLE_THRESHOLD) {
    currentState = STATE_SEARCH;
  } else {
    currentState = STATE_GOALKEEPER;
  }
}

void compassCorrection() {
  int target = 0;
  int error = target - angle;
  
  // Normalize error to [-180, 180]
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;

  if (abs(error) <= COMPASS_TOLERANCE) {
    stopMotors();
  } else if (error > 0) {
    turnRight(MOTOR_SPEED_MEDIUM);
  } else {
    turnLeft(MOTOR_SPEED_MEDIUM);
  }
}

void goalkeeperBehavior() {
  if (leftUltrasonic < ULTRASONIC_CLOSE) {
    seek10(MOTOR_SPEED_SLOW);
  }
  else if (leftUltrasonic > ULTRASONIC_FAR) {
    attack(MOTOR_SPEED_SLOW);
  }
  else if (backUltrasonic > ULTRASONIC_BACK_FAR) {
    seek10(MOTOR_SPEED_SLOW);
  }
  else if (backUltrasonic < ULTRASONIC_BACK_CLOSE) {
    attack(MOTOR_SPEED_SLOW);
  }
  else if (strongestEyeValue > 50) {
    // Ball tracking when weakly detected
    if (strongestEye >= 0 && strongestEye <= 2) turnRight(MOTOR_SPEED_MEDIUM);
    else if (strongestEye >= 6 && strongestEye <= 8) turnLeft(MOTOR_SPEED_MEDIUM);
    else if (strongestEye >= 3 && strongestEye <= 5) attack(MOTOR_SPEED_MEDIUM);
    else turnRight(MOTOR_SPEED_MEDIUM);
  }
}

// searchBehavior and attackBehavior remain unchanged
void searchBehavior() {
  switch (strongestEye) {
    case 4: case 3: case 5:
      attack(MOTOR_SPEED_FAST);
      break;
    case 2: 
      seek2(MOTOR_SPEED_MEDIUM);
      break;
    case 1:  
      seek1(MOTOR_SPEED_MEDIUM);
      break;
    case 6:
      seek6(MOTOR_SPEED_MEDIUM);
      break;
    case 8: case 9: 
      seek89(MOTOR_SPEED_MEDIUM);  
      break;
    case 7:
      seek7(MOTOR_SPEED_MEDIUM); 
    case 0: case 11: 
      seek011(MOTOR_SPEED_MEDIUM);  
      break;
    case 10: 
      seek10(MOTOR_SPEED_MEDIUM);  
      break;
    default:
      stopMotors();
      break;
  }
}
void attackBehavior() {
  switch (strongestEye) {
   case 4:case 5: case 3:
      attack(MOTOR_SPEED_FAST);
      break;
   case 2: 
      seek2(MOTOR_SPEED_FAST);
      break;
    case 1:  
      seek1(MOTOR_SPEED_FAST);
      break;
    case 6: 
      seek6(MOTOR_SPEED_FAST);
      break;
    case 8: case 9: 
      seek89(MOTOR_SPEED_FAST);  
      break;
    case 7:
      seek7(MOTOR_SPEED_FAST); 
    case 0: case 11: 
      seek011(MOTOR_SPEED_FAST);  
      break;
    case 10: 
      seek10(MOTOR_SPEED_FAST);  
      break;
    default:
      stopMotors();
      break;
  }
}
void protectBehavior() {
  seek10(MOTOR_SPEED_MEDIUM);
  
  // Add obstacle avoidance while backing up
  if (leftUltrasonic < ULTRASONIC_CLOSE || backUltrasonic < ULTRASONIC_BACK_CLOSE) {
    turnLeft(MOTOR_SPEED_FAST);
    delay(300);
  }
}

void debugOutput() {
  const char* stateNames[] = {"COMPASS", "ATTACK", "GOALKEEPER", "SEARCH", "PROTECT"};
  Serial.print("State: ");
  Serial.print(stateNames[currentState]);
  Serial.print(" | Angle: ");
  Serial.print(angle);
  Serial.print(" | EyeVal: ");
  Serial.print(strongestEyeValue);
  Serial.print(" | LeftUS: ");
  Serial.print(leftUltrasonic);
  Serial.print(" | BackUS: ");
  Serial.println(backUltrasonic);
}
uint32_t dt = 0;
void loop() {
  updateSensors();
  determineState();
if (angle>180 && angle<=270){
    setMotors(90,90,90,90);  
  Serial.println("a");
  }
  else if (angle > 270 && angle < 350){                   // If the car is facing left
    setMotors(50,50,50,50);                         // rotate clockwise
  Serial.println("b");
  }
  else if (angle > 10 && angle <= 90){              // If the car is facing right
   setMotors(-50,-50,-50,-50);        // rotate anti-clockwise
    
  Serial.println("c");
  }
  else if (angle > 90 && angle <= 180){              // If the car is facing right
    setMotors(-90,-90,-90,-90);
    
  Serial.println("d");
  }
  else{
//    setMotors(0,0,0,0);
//  switch (currentState) {
//    case STATE_ATTACK:
      attackBehavior();
//      break;
//    case STATE_GOALKEEPER:
//      goalkeeperBehavior();
//      break;
//    case STATE_SEARCH:
//      searchBehavior();
//      break;
//    case STATE_PROTECT:
//      protectBehavior();
//      break;
//  }
 }
 
  Serial.println(millis()-dt);
  dt = millis();
  debugOutput(); // Small delay for stability
}



/* Extra function
 *  
 * robot.moveSmart( a,b,c,d );  // motor move using compass as reference 
 *                              // a is angular direction, normally use compass reading as reference
 *                              // b is speed, motor car moving speed 
 *                              // c is angle, 
 *                              // d is precision,
 */