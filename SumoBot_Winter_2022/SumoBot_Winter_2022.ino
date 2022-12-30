#include <NHCSR04.h>
#include <Servo.h>

const int servo_scale = 150;
const int left_servo_zero = 1425;
const int right_servo_zero = 1370;

const int left_servo_min = left_servo_zero - servo_scale;
const int left_servo_max = left_servo_zero + servo_scale;

const int right_servo_min = right_servo_zero + servo_scale;
const int right_servo_max = right_servo_zero - servo_scale;

Servo left_servo;  // create servo object to control a servo
Servo right_servo;

int baudrate = 9600;

int centerTriggerPin = 2;
int centerEchoPin = 4;
int leftTriggerPin = 10;
int leftEchoPin = 16;
int rightTriggerPin = 7;
int rightEchoPin = 8;

const int sensorTimeout = 100;

SR04 center_sensor(centerTriggerPin, centerEchoPin, sensorTimeout);
SR04 left_sensor(leftTriggerPin, leftEchoPin, sensorTimeout);
SR04 right_sensor(rightTriggerPin, rightEchoPin, sensorTimeout);

const int left_edge_sensor_thresh = 890;
const int right_edge_sensor_thresh = 890;

void setup()
{
    Serial.begin(baudrate);
    left_servo.attach(6);  // attaches the servo on pin 9 to the servo object
    right_servo.attach(5);
    set_motors(0, 0);

    delay(4000);
}

void loop()
{
    double center_cm = center_sensor.centimeters();
    delay(10);
    double left_cm = left_sensor.centimeters();
    delay(10);
    double right_cm = right_sensor.centimeters();
    delay(10);

    // Serial.print(left_cm);
    // Serial.print(" ");
    // Serial.print(center_cm);
    // Serial.print(" ");
    // Serial.print(right_cm);
    // Serial.println("");


    double angle, magnitude;
    calculate_ultrasonic_vector(left_cm, center_cm, right_cm, angle, magnitude);

    bool left_sensor, right_sensor;
    get_edge_sensor_status(left_sensor, right_sensor);
    
    Serial.print(angle);
    Serial.println(" ");

    if (left_sensor == false && right_sensor == false)
    {
      int scale_factor;

      scale_factor = abs(angle) * 1.5;

      if (angle > 0)
      {
        set_motors(100 - scale_factor, 100);
      }
      else
      {
        set_motors(100, 100 - scale_factor);
      }
    }
    else
    {
      set_motors(-100, -100);
      delay(1000);
      if (left_sensor == false)
      {
        set_motors(100, -100);
        delay(700);
      }
      else
      {
        set_motors(-100, 100);
        delay(700);
      }
    }
}

void get_edge_sensor_status(bool& left_sensor, bool& right_sensor)
{
    int LeftEdgeSensor = analogRead(A0);
    int RightEdgeSensor = analogRead(A1);

    if (LeftEdgeSensor > left_edge_sensor_thresh) left_sensor = false;
    else left_sensor = true;

    if (RightEdgeSensor > right_edge_sensor_thresh) right_sensor = false;
    else right_sensor = true;
}

void calculate_ultrasonic_vector(double left_cm, double center_cm, double right_cm, double& angle, double& magnitude)
{
    double left_sensor_side_magnitude = -0.707 * left_cm;
    double left_sensor_forward_magnitude = 0.707 * left_cm;

    double right_sensor_side_magnitude = 0.707 * right_cm;
    double right_sensor_forward_magnitude = 0.707 * right_cm;

    double forward_sensor_forward_magnitude = center_cm;

    double forward_cm = (left_sensor_forward_magnitude + right_sensor_forward_magnitude + forward_sensor_forward_magnitude) / 3;
    double side_cm = (left_sensor_side_magnitude + right_sensor_side_magnitude);
    angle = (180.0 / 3.14159) * atan(side_cm/forward_cm);
    magnitude = sqrt(pow(forward_cm, 2.0) + pow(side_cm, 2.0));

}

void set_motors(int left_val, int right_val)
{
  // left_val = 0;
  // right_val = 0;

  int left_raw = map(left_val, -100, 100, left_servo_min, left_servo_max);
  int right_raw = map(right_val, -100, 100, right_servo_min, right_servo_max);

  left_servo.writeMicroseconds(left_raw);
  right_servo.writeMicroseconds(right_raw);
}
