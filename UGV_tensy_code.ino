#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>  // For stepper motor control

// =========================
// UGV DRIVE MOTOR CONFIG
// =========================
int mot1 = 9;
int mot2 = 7;
int mot3 = 33;
int mot4 = 37;
int pwm = 26000;
int pwm2 = 36000;
#define relay 34
bool camera1active = true;

// =========================
// Reverse Pins
int mr1= 11;
int mr2= 26;
int mr3= 28;
int mr4= 30;

// =========================
// MECHANISM CONFIG (PCA9685)
// =========================
#define m1pwm 8
#define m1dir 9
#define m2pwm 10
#define m2dir 11
#define m3pwm 12
#define m3dir 13
#define m4pwm 14
#define m4dir 15

int mechpwm = 4000;   // Default for motors 1 & 2
int mechpwm3 = 3600;  // Dedicated PWM for motor 3
int mechpwm4 = 2800;  // Dedicated PWM for motor 4
int runtime = 200;   
int runtime4 = 3000;  

// =========================
// AUGER STEPPER CONFIG
// =========================
#define STEP_PIN_1 22
#define DIR_PIN_1 21
#define STEP_PIN_2 23
#define DIR_PIN_2 20

const int stepsToMove = 6400;
const int stepSpeed = 32000;
const int stepAccel = 5000;

// Create two stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// =========================
// PCA HANDLER CLASS
// =========================
class pcahandler {
public:
  Adafruit_PWMServoDriver pwm;
  uint8_t i2cadd;
  TwoWire* i2cbus;
  bool intialized = false;

  pcahandler(TwoWire* bus, uint8_t addr = 0x40) {
    i2cadd = addr;
    i2cbus = bus;
    pwm = Adafruit_PWMServoDriver(addr, *i2cbus);
  }

  void beginwithretry() {
    pwm.begin();
    pwm.setPWMFreq(200);   // Better frequency for motor driver
    intialized = true;
    Serial.println("PCA9685 initialized on Wire2 (Pins 24/25)");
  }
};

// Create PCA instance
pcahandler mechPCA(&Wire2, 0x40);

// =========================
// MOTOR 3 TIMEOUT VARIABLES
// =========================
unsigned long lastMotor3Cmd = 0;
int motor3Timeout = 100;  // stop motor if no command for 100ms

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(57600);
  Serial4.begin(57600);

  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);
  pinMode(mot3, OUTPUT);
  pinMode(mot4, OUTPUT);

  pinMode(mr1,OUTPUT);
  pinMode(mr2,OUTPUT);
  pinMode(mr3,OUTPUT);
  pinMode(mr4,OUTPUT);
  
  analogWriteResolution(16);

  digitalWrite(13, HIGH);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);

  Wire2.begin();
  mechPCA.beginwithretry();

  // Stepper settings
  stepper1.setMaxSpeed(stepSpeed);
  stepper1.setAcceleration(stepAccel);
  stepper2.setMaxSpeed(stepSpeed);
  stepper2.setAcceleration(stepAccel);

  Serial.println("UGV + Auger system initialized");
  Serial.println("Drive: w/s/a/d/x | Mechanisms: 1f/1r..4f/4r/3s | Camera: c1/c2 | Auger: u1=up, u2=down, eo=on, ef=off");
}

// =========================
// LOOP
// =========================
void loop() {
  if (Serial4.available()) {
    handleInput(Serial4.readStringUntil('\n'), true);
  }
  if (Serial.available()) {
    handleInput(Serial.readStringUntil('\n'), false);
  }

  // Auto-stop Motor 3 if no new command recently
  if (millis() - lastMotor3Cmd > motor3Timeout) {
    motor3stop();
  }
}

// =========================
// COMMAND HANDLER
// =========================
void handleInput(String input, bool fromSerial4) {
  input.trim();

  if (fromSerial4) {
    Serial.print("Serial4: ");
  } else {
    Serial.print("USB: ");
  }
  Serial.println(input);

  if (input.length() == 1) {
    char c = input.charAt(0);
    if (c == 'w') forward();
    else if (c == 's') backward();
    else if (c == 'a') leftturn();
    else if (c == 'd') rightturn();
    else if (c == 'x') stopugv();
    else Serial.println("Unknown drive cmd");
  } else {
    // --- Mechanism controls ---
    // Motor 1: INVERTED
    if (input.equals("1f")) runmechtimed(m1pwm, m1dir, false);
    else if (input.equals("1r")) runmechtimed(m1pwm, m1dir, true);

    // Motor 2: normal timed
    else if (input.equals("2f")) runmechtimed(m2pwm, m2dir, true);
    else if (input.equals("2r")) runmechtimed(m2pwm, m2dir, false);

    // Motor 3: momentary (timeout based)
    else if (input.equals("3f")) {
      motor3forward();
      lastMotor3Cmd = millis();
    }
    else if (input.equals("3r")) {
      motor3reverse();
      lastMotor3Cmd = millis();
    }

    // Motor 4: 3s timed
    else if (input.equals("4f")) {
      runmechmotor(m4pwm, m4dir, true, mechpwm4);
      delay(runtime4);
      stopmechmotor(m4pwm);
      Serial.println("Motor 4 ran forward for 3 seconds");
    }
    else if (input.equals("4r")) {
      runmechmotor(m4pwm, m4dir, false, mechpwm4);
      delay(runtime4);
      stopmechmotor(m4pwm);
      Serial.println("Motor 4 ran reverse for 3 seconds");
    }

    // Camera relay
    else if (input.equals("c1")) {
      digitalWrite(relay, LOW);
      camera1active = true;
      Serial.println("Switched to Camera 1");
    } else if (input.equals("c2")) {
      digitalWrite(relay, HIGH);
      camera1active = false;
      Serial.println("Switched to Camera 2");
    }

    // --- Auger Stepper & Relay Commands ---
    else if (input.equals("u1")) {  // Auger UP
      moveBothSteppers(-stepsToMove);              
    } else if (input.equals("u2")) {  // Auger DOWN
      moveBothSteppers(stepsToMove);
    } else if (input.equals("eo")) {  // Auger Relay ON
      digitalWrite(relay, HIGH);
      Serial.println("Auger Relay ON");
    } else if (input.equals("ef")) {  // Auger Relay OFF
      digitalWrite(relay, LOW);
      Serial.println("Auger Relay OFF");
    } else {
      Serial.println("Unknown cmd");
    }
  }
}

// =========================
// DRIVE FUNCTIONS
// =========================
void forward() {
  digitalWrite(mr1,LOW);
  digitalWrite(mr2, LOW);
  analogWrite(mot1, pwm);
  analogWrite(mot2, pwm);
  analogWrite(mot3, 0);
  analogWrite(mot4, 0);
}
void backward() {
  digitalWrite(mr3, LOW);
  digitalWrite(mr4, LOW);
  analogWrite(mot1, 0);
  analogWrite(mot2, 0);
  analogWrite(mot3, pwm);
  analogWrite(mot4, pwm);
}
void leftturn() {
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
  digitalWrite(mr3,HIGH);
  digitalWrite(mr4,LOW);
  
  analogWrite(mot1, pwm);
  analogWrite(mot2, pwm);
  analogWrite(mot3, pwm);
  analogWrite(mot4, pwm);
}
void rightturn() {
  digitalWrite(mr1, LOW);
  digitalWrite(mr2, HIGH);
  digitalWrite(mr3, LOW);
  digitalWrite(mr4, HIGH);
  analogWrite(mot1, pwm);
  analogWrite(mot2, pwm);
  analogWrite(mot3, pwm);
  analogWrite(mot4, pwm);
}
void stopugv() {
  analogWrite(mot1, 0);
  analogWrite(mot2, 0);
  analogWrite(mot3, 0);
  analogWrite(mot4, 0);
}

// =========================
// MECHANISM FUNCTIONS
// =========================
void runmechmotor(uint8_t pwmch, uint8_t dirch, bool forward, int custompwm) {
  if (!mechPCA.intialized) {
    Serial.println("PCA9685 not initialized!");
    return;
  }
  if (forward) {
    mechPCA.pwm.setPWM(dirch, 4096, 0);   // HIGH
  } else {
    mechPCA.pwm.setPWM(dirch, 0, 4096);   // LOW
  }
  mechPCA.pwm.setPWM(pwmch, 0, custompwm);
}

void stopmechmotor(uint8_t pwmch) {
  if (!mechPCA.intialized) return;
  mechPCA.pwm.setPWM(pwmch, 0, 0);
}

void runmechtimed(uint8_t pwmch, uint8_t dirch, bool forward) {
  runmechmotor(pwmch, dirch, forward, mechpwm);
  delay(runtime);
  stopmechmotor(pwmch);
}

// =========================
// MOTOR 3 NEW FUNCTIONS
// =========================
void motor3forward() {
  runmechmotor(m3pwm, m3dir, true, mechpwm3);
}
void motor3reverse() {
  runmechmotor(m3pwm, m3dir, false, mechpwm3);
}
void motor3stop() {
  stopmechmotor(m3pwm);
}

// =========================
// AUGER STEPPER FUNCTION
// =========================
void moveBothSteppers(int steps) {
  stepper1.move(steps);
  stepper2.move(steps);
  while (stepper1.isRunning() || stepper2.isRunning()) {
    stepper1.run();
    stepper2.run();
  }
}
