/* Straight line drive code using MPU6050 IMU */
//HEADER FILES
#include <MPU6050_tockn.h>
#include <Wire.h>

//PIN CONFIG

//I2C CONFIG
#define SDA 21
#define SCL 22

//MOTOR A OR LEFT MOTOR
const int ain1_pin = 23;
const int ain2_pin = 12;
const int pwma_pin = 14;

//MOTOR B OR RIGHT
const int bin1_pin = 5;
const int bin2_pin = 18;
const int pwmb_pin = 19;

// PWM CONFIG
const int freq = 30000;
const int pwmChannel_l = 0;
const int pwmChannel_r = 1;
const int resolution = 8;
int motor_power = 190;
int turn_speed = 200
;

//GLOBAL VARIABLES
MPU6050 mpu6050(Wire);
float x = 0;
float z_calibrated = 0; //calibrated z value
float gyro_z = 0; //current z value
double leftSpeed = 0;
double rightSpeed = 0;
double yaw = 0;
double Kp = 0.2;
double gyro_turn_val = 90;

double robo_turn_val = 0;

double current_left_turn_val = 0;

double current_right_turn_val = 0;

const int encoder_turn_val = 8;


void straight_line_drive_init()
{
  // PIN MODE SELECT
  pinMode(pwma_pin, OUTPUT);
  pinMode(ain1_pin, OUTPUT);
  pinMode(ain2_pin, OUTPUT);
  pinMode(pwmb_pin, OUTPUT);
  pinMode(bin1_pin, OUTPUT);
  pinMode(bin2_pin, OUTPUT);

  // ANALOG WRITE (LEDC) SETUP
  ledcSetup(pwmChannel_l, freq, resolution);
  ledcSetup(pwmChannel_r, freq, resolution);

  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(pwma_pin, pwmChannel_l);
  ledcAttachPin(pwmb_pin, pwmChannel_r);

  Wire.begin(SDA, SCL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); //Gyroscope Calibration
  z_calibrated = mpu6050.getGyroZoffset();
}

void yaw_loop()
{
  mpu6050.update();
  gyro_z = mpu6050.getGyroAngleZ() - x;

  yaw = z_calibrated - gyro_z;

  leftSpeed = motor_power - yaw;
  rightSpeed = motor_power + yaw;

  ledcWrite(pwmChannel_l, leftSpeed);
  ledcWrite(pwmChannel_r, rightSpeed);
}

void startMotors(boolean en)
{
  if ( en )
  {
    ledcWrite(pwmChannel_l, motor_power);
    ledcWrite(pwmChannel_r, motor_power);
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, HIGH);
    digitalWrite(bin1_pin, HIGH);
    digitalWrite(bin2_pin, LOW);
  }
  else
  {
    ledcWrite(pwmChannel_l, 0);
    ledcWrite(pwmChannel_r, 0);
    digitalWrite(ain1_pin, LOW);
    digitalWrite(ain2_pin, LOW);
    digitalWrite(bin1_pin, LOW);
    digitalWrite(bin2_pin, LOW);
  }
}
void turnMotor_right()
{
  startMotors(false);
  lenc = 0;
  renc = 0;
  pcnt_counter_clear(PCNT_TEST_UNIT_1);
  pcnt_counter_clear(PCNT_TEST_UNIT_2);
  Serial.println("Turning Right");
  ledcWrite(pwmChannel_l, turn_speed);
  digitalWrite(ain1_pin, LOW);
  digitalWrite(ain2_pin, HIGH);
  //mqtt_publish(pubsubClient, "Robo2", "Turning motor right");
  mpu6050.update();
  while ((mpu6050.getGyroAngleZ()-x) >= (-170 - z_calibrated))
  {
    mpu6050.update();
  }
  startMotors(false);
  pcnt_counter_clear(PCNT_TEST_UNIT_1);
  pcnt_counter_resume(PCNT_TEST_UNIT_1);
  pcnt_counter_clear(PCNT_TEST_UNIT_2);
  pcnt_counter_resume(PCNT_TEST_UNIT_2);
  lenc=0;
  renc=0;
  //gyro recalibrate
  mpu6050.calcGyroOffsets(true); //Gyroscope Calibration
  mpu6050.update();
  x = mpu6050.getGyroAngleZ();
}
void turnMotor_left()
{
  startMotors(false);
  Serial.println("Turning Left");
  //turn on right motor
  ledcWrite(pwmChannel_r, turn_speed);
  digitalWrite(bin1_pin, HIGH);
  digitalWrite(bin2_pin, LOW );
  //mqtt_publish(pubsubClient, "Robo2", "Turning motor left");
  while ((mpu6050.getGyroAngleZ()-x) <= (170 + z_calibrated))
  {
    mpu6050.update();
  }
  startMotors(false);
  pcnt_counter_clear(PCNT_TEST_UNIT_1);
  pcnt_counter_resume(PCNT_TEST_UNIT_1);
  pcnt_counter_clear(PCNT_TEST_UNIT_2);
  pcnt_counter_resume(PCNT_TEST_UNIT_2);
  lenc=0;
  renc=0;
  //gyro recalibrate
  mpu6050.calcGyroOffsets(true); //Gyroscope Calibration
  mpu6050.update();
  x = mpu6050.getGyroAngleZ();
}
