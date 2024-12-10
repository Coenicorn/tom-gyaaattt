#include <AccelStepper.h>

#define STEP_PIN1 6
#define DIR_PIN1 5

/* THIS IS THE MOTOR AT THE BOTTOM */
#define MOTOR_X_SWITCH_PIN 23
#define MOTOR_X_MAX_RANGE_DEG 191.69f

// I counted -_-
#define GEARS_BIG (float)(61)
#define GEARS_SMOL (float)(14)
#define GEAR_RATIO (float)(GEARS_BIG / GEARS_SMOL)

#define STEPS_PER_DEG (200.f / 360.f * GEAR_RATIO)
// convert degrees to stepper motor steps to be used by AccelStepper.moveTo()
// AccelStepper.moveTo(D2S(180))
#define D2S(_D) ((float)_D * STEPS_PER_DEG)

AccelStepper motorX(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);

void initializeMotors();
void homingSequence(unsigned int speed, AccelStepper &motor, uint8_t switch_oin /* MAKE SURE THIS IS CORRECT */);
void motorGoTo(long absolute, AccelStepper &motor, float speed);

void setup()
{
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);

  motorX.setMaxSpeed(500);
  motorX.setAcceleration(1);

  pinMode(MOTOR_X_SWITCH_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  initializeMotors();
}

void loop()
{

  // motorX.moveTo(omzetten_x);
  // motorX.run();

  // while (motorX.distanceToGo() != 0 ) {
  //   motorX.run();

  // }

  // delay(3000);

  // motorX.moveTo(0);

  // while (motorX.distanceToGo() != 0 ) {
  //   motorX.run();

  //  }
  //  delay(3000);


  delay(1000);
  motorGoTo(D2S(180), motorX, 500);
  delay(1000);
  motorGoTo(D2S(10), motorX, 500);
}

void motorGoTo(long absolute /* position to go to IN STEPS */, AccelStepper &motor, float speed)
{

  motor.moveTo(absolute);
  motor.setSpeed(speed);

  while (motor.distanceToGo() != 0)
    motor.runSpeedToPosition();
}

void homingSequence(unsigned int _speed, AccelStepper &motor, uint8_t switch_pin /* MAKE SURE THIS IS CORRECT */)
{
  float speed = -((float)_speed);

  Serial.print("homing with speed ");
  Serial.println(speed);

  // un-click trigger if it is already triggered
  motorGoTo(45, motor, 200);

  motor.setSpeed(speed);

  while (digitalRead(switch_pin) == HIGH)
    motor.runSpeed();

  motor.stop();

  motor.setCurrentPosition(0);
}

void initializeMotors()
{
  Serial.println("Initializing motors...");

  motorX.setCurrentPosition(0);

  homingSequence(200, motorX, MOTOR_X_SWITCH_PIN);

  Serial.println("Initialization complete. Let’s transform and ascend!");

  delay(1000);

  // disable to save power
  motorX.disableOutputs();
}