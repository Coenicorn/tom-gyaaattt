#include <AccelStepper.h>

#define STEP_PIN1 15
#define DIR_PIN1 16


#define STEP_PIN2 12
#define DIR_PIN2 13

/* THIS IS THE MOTOR AT THE TOP */
#define MOTOR_Y_SWITCH_PIN 18
/* THIS IS THE MOTOR AT THE BOTTOM */
#define MOTOR_X_SWITCH_PIN 23

#define BUT_PIN_FUNC1 19
#define BUT_PIN_FUNC2 21

#define MOTOR_X_MAX_RANGE_DEG 604.80f
#define MOTOR_Y_MAX_RANGE_DEG 191.69f

#define MOTOR_X_MAX_RANGE_STEPS 1464
#define MOTOR_Y_MAX_RANGE_STEPS 464

// I counted -_-
#define GEARS_BIG (float)(61)
#define GEARS_SMOL (float)(14)
#define GEAR_RATIO (float)(GEARS_BIG / GEARS_SMOL)

#define STEPS_PER_DEG (200.f / 360.f * GEAR_RATIO)
// convert degrees to stepper motor steps to be used by AccelStepper.moveTo()
// AccelStepper.moveTo(D2S(180))
#define D2S(_D) ((float)_D * STEPS_PER_DEG)

AccelStepper motorX(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper motorY(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

int omzetten_x;
int omzetten_y;

void initializeMotors();
void homingSequence(unsigned int speed, AccelStepper &motor, uint8_t switch_oin /* MAKE SURE THIS IS CORRECT */);
void motorGoTo(long absolute, AccelStepper &motor, float speed);

void setup()
{

  int x = 180; // limiet op 100(360 graden) naarmate initialisatie word gedaan
  int y = 20;  // limiet draaien zit op 40(144 graden) naarmate initialisatie goed word gedaan
               // 180 graden is 2.62, 360 graden is 2.32, 90 is 2.70
  omzetten_x = x * 2.62;
  omzetten_y = y * 2.40;

  motorX.setMaxSpeed(200);
  motorX.setAcceleration(1);
  motorY.setMaxSpeed(200);
  motorY.setAcceleration(1);

  pinMode(MOTOR_X_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_Y_SWITCH_PIN, INPUT_PULLUP);

  pinMode(BUT_PIN_FUNC1, INPUT_PULLUP);
  pinMode(BUT_PIN_FUNC2, INPUT_PULLUP);

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
  motorY.setCurrentPosition(0);

  homingSequence(200, motorX, MOTOR_X_SWITCH_PIN);
  homingSequence(200, motorY, MOTOR_Y_SWITCH_PIN);

  Serial.println("Initialization complete. Let’s transform and ascend!");

  delay(1000);

  // disable to save power
  motorX.disableOutputs();
  motorY.disableOutputs();
}