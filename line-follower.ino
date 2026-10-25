/************** PIN DEFINITIONS **************/
#define STBY 4

// LEFT MOTOR (TB6612FNG)
#define PWMA 5
#define AIN1 9
#define AIN2 10

// RIGHT MOTOR (TB6612FNG)
#define PWMB 6
#define BIN1 7
#define BIN2 8

/************** IR SENSORS **************/
const uint8_t sensorPins[5] = {A0, A1, A2, A3, A4};

/************** MAIN TUNING (YOU ADJUST) **************/
float Kp = 32.0;
float Kd = 16.0;
int BASE_SPEED = 110;
int MAX_SPEED  = 255;
int THRESHOLD  = 600;

/************** LOOP/CORNER PROTECTION KNOBS **************/
// Slows down automatically on big error -> helps loops
int CORNER_SLOW = 65;                 // 55..85 safe

// Smooth PWM only in normal mode
const int PWM_SLEW = 10;               // 8..12 safe

// Pivot (sharp 90/acute)
int PIVOT_ERR = 1450;                 // 1400..1650 safe
int PIVOT_PWM = 175;                  // 165..205 safe
int PIVOT_HOLD_MS = 150;              // 120..190 safe

// --- IMPORTANT: loop protection ---
// If center sensor is still strongly black, it's likely a curve/loop -> don't pivot
const int PIVOT_CENTER_BLOCK = 320;   // 300..380 safe

// Prevent double pivots (unwanted 180 at joints)
const int PIVOT_COOLDOWN_MS = 300;    // 260..400 safe

// Lost creep (only when NOT pivoting)
const int LOST_CREEP = 60;            // 45..70 safe
const int LOST_BIAS  = 35;            // 20..40 safe

// Center sensor black threshold for ending pivot (line reacquired)
const int CENTER_BLACK_OK = 220;      // 180..260 safe

/************** FINISH / BRANCH CUT ASSIST **************/
// Helps bot take a small left-cut near finish instead of going straight.
const int BRANCH_BLACK    = 260;      // 220..320 (sensitivity)
const int BRANCH_BIAS_PWM = 35;       // 20..60  (commit strength)
const int BRANCH_HOLD_MS  = 90;       // 60..140 (commit time)

/************** STOP BOX DETECTION (NEW) **************/
// Stop when all 5 sensors see strong black for some time.
const int STOP_BLACK      = 260;      // 200..400 (how strong black must be)
const int STOP_HOLD_MS    = 180;      // 120..300 (debounce time)

/************** INTERNALS **************/
int blackVal[5] = {0,0,0,0,0};

int lastError = 0;
int lastLeftPWM = 0;
int lastRightPWM = 0;

// Pivot state
bool pivoting = false;
int pivotDir = 0;                     // -1 = left, +1 = right
unsigned long pivotStart = 0;

// Cooldown state
unsigned long pivotCooldownUntil = 0;

// Branch assist state
unsigned long branchUntil = 0;

// Stop-box timer
unsigned long stopStart = 0;

/************** MOTOR SIGNED CONTROL **************/
// pwm: -255..+255 (negative = reverse)
void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -pwm);
  }
}

void setRightMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, pwm);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -pwm);
  }
}

int applySlew(int current, int target, int step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

/************** NORMAL DRIVE (FORWARD ONLY, SMOOTHED) **************/
void driveSmoothNormal(int leftTarget, int rightTarget) {
  leftTarget  = constrain(leftTarget,  0, MAX_SPEED);
  rightTarget = constrain(rightTarget, 0, MAX_SPEED);

  lastLeftPWM  = applySlew(lastLeftPWM,  leftTarget,  PWM_SLEW);
  lastRightPWM = applySlew(lastRightPWM, rightTarget, PWM_SLEW);

  // Force forward in normal mode
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, lastLeftPWM);
  analogWrite(PWMB, lastRightPWM);
}

/************** TRUE PIVOT (FORWARD + REVERSE, NO SLEW) **************/
void pivotSpin(int dir) {
  if (dir > 0) {  // RIGHT
    setLeftMotor(+PIVOT_PWM);
    setRightMotor(-PIVOT_PWM);
  } else {        // LEFT
    setLeftMotor(-PIVOT_PWM);
    setRightMotor(+PIVOT_PWM);
  }
}

/************** SENSOR LOGIC **************/
// returns position 0..4000, or -1 if lost. Also fills blackVal[].
int readLinePosition() {
  long sum = 0;
  long weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    int v = analogRead(sensorPins[i]);
    int black = THRESHOLD - v;   // black is LOW -> strength positive

    if (black < 0) black = 0;
    if (black > 1000) black = 1000;

    blackVal[i] = black;
    sum += black;
    weightedSum += (long)black * (i * 1000);
  }

  if (sum == 0) return -1;
  return (int)(weightedSum / sum);
}

bool centerOnLine() {
  return (blackVal[2] > CENTER_BLACK_OK);
}

/************** STOP BOX CHECK **************/
bool allSensorsBlackStrong() {
  for (int i = 0; i < 5; i++) {
    if (blackVal[i] < STOP_BLACK) return false;
  }
  return true;
}

/************** LOST LINE RECOVERY **************/
void recoverLostLineCreep() {
  if (lastError < 0) {
    driveSmoothNormal(LOST_CREEP - LOST_BIAS, LOST_CREEP + LOST_BIAS);
  } else {
    driveSmoothNormal(LOST_CREEP + LOST_BIAS, LOST_CREEP - LOST_BIAS);
  }
}

/************** SETUP **************/
void setup() {
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, HIGH);

  setLeftMotor(0);
  setRightMotor(0);
  delay(200);
}

/************** LOOP **************/
void loop() {
  int position = readLinePosition();

  // --- STOP BOX DETECTION (works in any mode) ---
  if (allSensorsBlackStrong()) {
    if (stopStart == 0) stopStart = millis();
    if (millis() - stopStart >= (unsigned long)STOP_HOLD_MS) {
      setLeftMotor(0);
      setRightMotor(0);
      while (1); // stop forever
    }
  } else {
    stopStart = 0;
  }

  /***** 1) PIVOT LOCK MODE *****/
  if (pivoting) {
    pivotSpin(pivotDir);

    if (centerOnLine() || (millis() - pivotStart > (unsigned long)PIVOT_HOLD_MS)) {
      pivoting = false;

      pivotCooldownUntil = millis() + (unsigned long)PIVOT_COOLDOWN_MS;

      lastLeftPWM = 0;
      lastRightPWM = 0;

      branchUntil = 0;
    }
    return;
  }

  bool inCooldown = (millis() < pivotCooldownUntil);

  /***** 2) LOST LINE (NOT pivoting) *****/
  if (position == -1) {
    if (inCooldown) {
      setLeftMotor(0);
      setRightMotor(0);
      return;
    }

    if (abs(lastError) > PIVOT_ERR) {
      pivoting = true;
      pivotDir = (lastError > 0) ? +1 : -1;
      pivotStart = millis();
      return;
    }

    recoverLostLineCreep();
    return;
  }

  /***** 3) BRANCH / FINISH CUT ASSIST *****/
  bool nearStraight = (abs(lastError) < 250);
  bool leftBranch   = (blackVal[0] > BRANCH_BLACK || blackVal[1] > BRANCH_BLACK);

  if (!inCooldown && nearStraight && leftBranch) {
    branchUntil = millis() + (unsigned long)BRANCH_HOLD_MS;
  }

  if (millis() < branchUntil) {
    int leftTarget  = BASE_SPEED - BRANCH_BIAS_PWM;
    int rightTarget = BASE_SPEED + BRANCH_BIAS_PWM;
    driveSmoothNormal(leftTarget, rightTarget);
    return;
  }

  /***** 4) NORMAL PD CONTROL *****/
  // ✅ direction for your bot: left -> negative, right -> positive
  int error = position - 2000;

  int derivative = error - lastError;
  lastError = error;

  int absErr = abs(error);

  int base = BASE_SPEED - map(constrain(absErr, 0, 2000), 0, 2000, 0, CORNER_SLOW);
  base = constrain(base, 85, BASE_SPEED);

  bool centerStrong = (blackVal[2] > PIVOT_CENTER_BLOCK);

  if (!inCooldown && !centerStrong && absErr > PIVOT_ERR) {
    pivoting = true;
    pivotDir = (error > 0) ? +1 : -1;
    pivotStart = millis();
    return;
  }

  int correction = (int)(Kp * error + Kd * derivative);

  int leftTarget  = base + correction;
  int rightTarget = base - correction;

  driveSmoothNormal(leftTarget, rightTarget);
}
