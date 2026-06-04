// Candy serving + obstacle avoidance (4 ultrasonics; no Servo = PWM on D9 works).
// Standard OSOYOO: ENA->D9, IN1->D12, IN2->D11 | ENB->D6, IN3->D7, IN4->D8
// Front 1: TRIG D10 ECHO D2  |  Front 2: TRIG D0 ECHO D1 (unplug 0/1 when uploading)
// Left D4/D3 | Right D5/D13
// Load learning: gSpeedMul slowly raises PWM on high motor demand (carpet) and lowers on
// light drag (tile). Best with optional current sense on A4; else uses closing speed on a wall ahead.
// To use Serial Monitor: unplug Front2 (pins 0,1) then open Serial at 9600

// ---------- Motor pins (standard - both tyres move) ----------
#define speedPinR 9
#define RightMotorDirPin1 12
#define RightMotorDirPin2 11
#define speedPinL 6
#define LeftMotorDirPin1  7
#define LeftMotorDirPin2  8

// ---------- Ultrasonic sensor pins ----------
const int TRIG_FRONT  = 10;
const int ECHO_FRONT  = 2;
const int TRIG_FRONT2 = 0;   // second front (unplug for upload)
const int ECHO_FRONT2 = 1;

const int TRIG_LEFT  = 4;
const int ECHO_LEFT  = 3;

const int TRIG_RIGHT = 5;
const int ECHO_RIGHT = 13;

// ---------- Settings ----------
const int MOTOR_SPEED     = 110;   // forward base (0–255); scaled by gSpeedMul
const int TURN_SPEED      = 90;   // backup / side nudge (slower = smoother)
const int TURN_FAST       = 95;   // obstacle side turns (still scaled by gSpeedMul)
const int SAFE_FRONT_CM   = 58;   // detect front obstacle earlier (more distance)
const int SAFE_LEFT_CM    = 40;   // left side safe distance
const int SAFE_RIGHT_CM   = 40;   // right side safe distance
// Second front on D0/D1 often reads garbage → false obstacle / stop–go. Use primary only unless both wired well.
const bool USE_SECOND_FRONT_SENSOR = false;
const int FRONT_DEBOUNCE_SAMPLES = 2;  // consecutive “close” reads before reacting
const unsigned long PING_TIMEOUT_US = 20000UL;
const bool PRINT_SERIAL   = false;
const int SIDE_COOLDOWN_LOOPS = 12;  // shorter ignore window = faster side re-detection
const int SIDE_PAUSE_MS  = 800;      // shorter pause for faster side response
// true = 3 s tray pause + slow long pivot (candy). false = quick backup + fast turn (reliable avoid).
const bool CANDY_LONG_SERVE = false;
const int FRONT_PAUSE_SERVE_MS = 3000;
const int FRONT_PAUSE_QUICK_MS = 400;

// ---------- Slow serve turn (only if CANDY_LONG_SERVE; tune ms for ~180°) ----------
const int TURN_SERVE_PWM = 88;
const unsigned long SERVE_TURN_LEFT_MS  = 3200UL;
const unsigned long SERVE_TURN_RIGHT_MS = 3100UL;
// ---------- Quick front avoid (if !CANDY_LONG_SERVE) ----------
const int FRONT_BACK_MS = 380;
const int TURN_FAST_FRONT_MS = 720;
const int TURN_FAST_SIDE_MS  = 520;  // wider left/right side avoid turn

// ---------- Load learning (all motor PWM multiplied by gSpeedMul) ----------
float gSpeedMul = 1.0f;
const float SPEED_MUL_MIN = 0.68f;
const float SPEED_MUL_MAX = 1.42f;
const float LOAD_ADAPT_STEP = 0.015f;
const unsigned long LOAD_ADAPT_PERIOD_MS = 550UL;
// Set true if A4 is wired to driver/shunt so voltage rises under load (calibrate thresholds).
const bool USE_MOTOR_CURRENT_SENSE = false;
const int MOTOR_CURRENT_PIN = A4;
int gCurrentIdleAnalog = 512;
const int CURRENT_HIGH_OVER_IDLE = 85;   // raise mul when above this (heavy / carpet)
const int CURRENT_LOW_OVER_IDLE  = 28;   // lower mul when load signal is low (easy floor)

// ---------- Distance: one ping, return cm (big number if no echo) ----------
long readDistanceCm(int trigPin, int echoPin) {
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, PING_TIMEOUT_US);
  if (duration == 0) return 400;          // no echo -> treat as far away

  long cm = (duration + 29) / 58;         // HC‑SR04 formula
  if (cm < 2 || cm > 400) return 400;     // out of range -> far
  return cm;
}

// Front distance = minimum of both front sensors (better accuracy)
long readFrontCm() {
  long d1a = readDistanceCm(TRIG_FRONT, ECHO_FRONT);
  delay(6);
  long d2  = readDistanceCm(TRIG_FRONT2, ECHO_FRONT2);
  delay(6);
  long d1b = readDistanceCm(TRIG_FRONT, ECHO_FRONT);
  long d1 = (d1a < d1b) ? d1a : d1b;
  return (d1 < d2) ? d1 : d2;
}

// ---------- Load adaptation (call only while cruising straight) ----------
static long sLoadPrevFrontCm = -1;
static unsigned long sLoadLastAdaptMs = 0;

void loadAdaptReset() {
  sLoadPrevFrontCm = -1;
  sLoadLastAdaptMs = 0;
}

void updateLoadAdaptWhileCruising(long dFront) {
  unsigned long now = millis();
  if (sLoadLastAdaptMs == 0) {
    sLoadLastAdaptMs = now;
    sLoadPrevFrontCm = dFront;
    return;
  }
  if (now - sLoadLastAdaptMs < LOAD_ADAPT_PERIOD_MS) return;
  unsigned long dt = now - sLoadLastAdaptMs;
  sLoadLastAdaptMs = now;

  if (USE_MOTOR_CURRENT_SENSE) {
    int raw = analogRead(MOTOR_CURRENT_PIN);
    static int ema = -1;
    if (ema < 0) ema = raw;
    else ema = (ema * 7 + raw) / 8;
    int over = ema - gCurrentIdleAnalog;
    if (over > CURRENT_HIGH_OVER_IDLE) {
      gSpeedMul += LOAD_ADAPT_STEP;
    } else if (over < CURRENT_LOW_OVER_IDLE) {
      gSpeedMul -= LOAD_ADAPT_STEP * 0.65f;
    }
  } else if (sLoadPrevFrontCm >= 0) {
    bool inBand = (dFront > (SAFE_FRONT_CM + 38)) && (dFront < 210);
    if (inBand) {
      float closing = (sLoadPrevFrontCm - (float)dFront) * 1000.0f / (float)dt;
      if (closing > 26.0f) {
        gSpeedMul -= LOAD_ADAPT_STEP * 0.72f;
      } else if (closing < 11.0f && closing > -4.0f) {
        gSpeedMul += LOAD_ADAPT_STEP;
      }
    }
  }
  sLoadPrevFrontCm = dFront;
  if (gSpeedMul < SPEED_MUL_MIN) gSpeedMul = SPEED_MUL_MIN;
  if (gSpeedMul > SPEED_MUL_MAX) gSpeedMul = SPEED_MUL_MAX;
}

// ---------- Motor control ----------
// Cruise forward uses gSpeedMul (carpet learning). Turns / backup use RAW PWM so low mul cannot stall pivots.
void set_MotorspeedRaw(int speed_L, int speed_R) {
  speed_L = constrain(speed_L, 0, 255);
  speed_R = constrain(speed_R, 0, 255);
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);
}

void set_Motorspeed(int speed_L, int speed_R) {
  float m = gSpeedMul;
  speed_L = constrain((int)(speed_L * m + 0.5f), 0, 255);
  speed_R = constrain((int)(speed_R * m + 0.5f), 0, 255);
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);
}

void stop_Stop() {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
  set_Motorspeed(0, 0);
}

void go_Advance() {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  set_Motorspeed(MOTOR_SPEED, MOTOR_SPEED);
}

void go_Back(int t) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  set_MotorspeedRaw(TURN_SPEED, TURN_SPEED);
  if (t > 0) delay(t);
}

void go_Left(int t) {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  set_MotorspeedRaw(TURN_SPEED, TURN_SPEED);
  if (t > 0) delay(t);
}

void go_Right(int t) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  set_MotorspeedRaw(TURN_SPEED, TURN_SPEED);
  if (t > 0) delay(t);
}

// Fast turn (for obstacle avoid only)
void turnFastLeft(int t) {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  set_MotorspeedRaw(TURN_FAST, TURN_FAST);
  if (t > 0) delay(t);
}
void turnFastRight(int t) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  set_MotorspeedRaw(TURN_FAST, TURN_FAST);
  if (t > 0) delay(t);
}

// Slow in-place pivot after serving (candy-safe; shorten *_MS if it turns too far).
void turnServeLeft(unsigned long ms) {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  set_MotorspeedRaw(TURN_SERVE_PWM, TURN_SERVE_PWM);
  delay(ms);
}

void turnServeRight(unsigned long ms) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  set_MotorspeedRaw(TURN_SERVE_PWM, TURN_SERVE_PWM);
  delay(ms);
}

// ---------- Setup ----------
void setup() {
  // Motors (OSOYOO init_GPIO style)
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(speedPinR, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_FRONT2, OUTPUT);
  pinMode(ECHO_FRONT2, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(MOTOR_CURRENT_PIN, INPUT);
  delay(100);
  gCurrentIdleAnalog = analogRead(MOTOR_CURRENT_PIN);

  if (PRINT_SERIAL) Serial.begin(9600);
  stop_Stop();
}

// ---------- Main loop: front and sides use same logic (detect → react → move straight) ----------
void loop() {
  static int leftCooldown  = 0;  // ignore left sensor for this many loops after turning for left obstacle
  static int rightCooldown = 0;  // ignore right sensor after turning for right obstacle
  static int frontCloseStreak = 0;

  long dFront1 = readDistanceCm(TRIG_FRONT, ECHO_FRONT);
  long dFront2 = readDistanceCm(TRIG_FRONT2, ECHO_FRONT2);
  long dLeft   = readDistanceCm(TRIG_LEFT,  ECHO_LEFT);
  long dRight  = readDistanceCm(TRIG_RIGHT, ECHO_RIGHT);

  long dFront = USE_SECOND_FRONT_SENSOR ? ((dFront1 < dFront2) ? dFront1 : dFront2) : dFront1;

  if (dFront <= SAFE_FRONT_CM) {
    if (frontCloseStreak < FRONT_DEBOUNCE_SAMPLES) frontCloseStreak++;
  } else {
    frontCloseStreak = 0;
  }
  int frontObstacle = (frontCloseStreak >= FRONT_DEBOUNCE_SAMPLES) ? 1 : 0;

  // Do not creep forward while front is “close” but not yet debounced (prevents bump–stop–bump).
  if (!frontObstacle && dFront <= SAFE_FRONT_CM) {
    stop_Stop();
    delay(10);
    return;
  }

  if (leftCooldown  > 0) leftCooldown--;
  if (rightCooldown > 0) rightCooldown--;

  int leftClose  = (dLeft  < SAFE_LEFT_CM) && (leftCooldown  == 0) ? 1 : 0;
  int rightClose = (dRight < SAFE_RIGHT_CM) && (rightCooldown == 0) ? 1 : 0;

  if (PRINT_SERIAL) {
    Serial.print("F1:");
    Serial.print(dFront1);
    Serial.print(" F2:");
    Serial.print(dFront2);
    Serial.print(" L:");
    Serial.print(dLeft);
    Serial.print(" R:");
    Serial.println(dRight);
  }

  if (frontObstacle) {
    loadAdaptReset();
    frontCloseStreak = 0;
    stop_Stop();
    if (CANDY_LONG_SERVE) {
      delay(FRONT_PAUSE_SERVE_MS);
      stop_Stop();
      delay(20);
      if (dRight > dLeft) {
        turnServeRight(SERVE_TURN_RIGHT_MS);
      } else {
        turnServeLeft(SERVE_TURN_LEFT_MS);
      }
    } else {
      delay(FRONT_PAUSE_QUICK_MS);
      go_Back(FRONT_BACK_MS);
      stop_Stop();
      delay(20);
      if (dRight > dLeft) {
        turnFastRight(TURN_FAST_FRONT_MS);
      } else {
        turnFastLeft(TURN_FAST_FRONT_MS);
      }
    }
    stop_Stop();
    delay(25);
    go_Advance();
    delay(220);
    stop_Stop();
    delay(20);
  } else if (leftClose && (!rightClose || dRight > dLeft)) {
    loadAdaptReset();
    // ---------- LEFT OBSTACLE: turn right, then cooldown LEFT so we drive straight ----------
    stop_Stop();
    delay(SIDE_PAUSE_MS);
    turnFastRight(TURN_FAST_SIDE_MS);
    stop_Stop();
    delay(15);
    go_Advance();
    delay(180);
    stop_Stop();
    leftCooldown = SIDE_COOLDOWN_LOOPS;
  } else if (rightClose && (!leftClose || dLeft > dRight)) {
    loadAdaptReset();
    // ---------- RIGHT OBSTACLE: turn left, then cooldown RIGHT so we drive straight ----------
    stop_Stop();
    delay(SIDE_PAUSE_MS);
    turnFastLeft(TURN_FAST_SIDE_MS);
    stop_Stop();
    delay(15);
    go_Advance();
    delay(180);
    stop_Stop();
    rightCooldown = SIDE_COOLDOWN_LOOPS;
  } else if (leftClose && rightClose) {
    loadAdaptReset();
    // ---------- BOTH SIDES CLOSE: turn toward clearer side, cooldown that side ----------
    stop_Stop();
    delay(SIDE_PAUSE_MS);
    if (dRight >= dLeft) {
      turnFastRight(TURN_FAST_SIDE_MS);
      stop_Stop();
      delay(15);
      go_Advance();
      delay(180);
      stop_Stop();
      leftCooldown = SIDE_COOLDOWN_LOOPS;
    } else {
      turnFastLeft(TURN_FAST_SIDE_MS);
      stop_Stop();
      delay(15);
      go_Advance();
      delay(180);
      stop_Stop();
      rightCooldown = SIDE_COOLDOWN_LOOPS;
    }
  } else {
    // ---------- ALL CLEAR: drive straight; slowly learn carpet vs tile ----------
    go_Advance();
    updateLoadAdaptWhileCruising(dFront);
  }

  delay(10);
}