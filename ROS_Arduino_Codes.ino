#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <Wire.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>

const int echoPinFront = 5;
const int trigPinFront = A3;

const int echoPinRight = 4;
const int trigPinRight = A2;

const int echoPinLeft = 2;
const int trigPinLeft = A1;

const int MPU_addr = 0x68; 
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;


std_msgs::String imu_msg;
ros::Publisher imu("imu", &imu_msg);
ros::NodeHandle nh;


#define EN_L 9
#define IN1_L 10
#define IN2_L 11
#define EN_R 8
#define IN1_R 12
#define IN2_R 13
double w_r = 0, w_l = 0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.295;

//ros::NodeHandle nh;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang = 0, speed_lin = 0;
/* END */


std_msgs::Int16 msg;
ros::Publisher Front_pub("front", &msg);

std_msgs::Int16 Left_msg;
ros::Publisher Left_pub("left", &Left_msg);

std_msgs::Int16 Right_msg;
ros::Publisher Right_pub("right", &Right_msg);

void messageCb( const geometry_msgs::Twist& msg) {
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));

}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

long duration;
int distance;

void setup()
{

  pinMode(trigPinFront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinFront, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinRight, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRight, INPUT); // Sets the echoPin as an Input

  pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT); // Sets the echoPin as an Input

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  nh.initNode();
  nh.advertise(imu);
  Motors_init();
  nh.subscribe(sub);
  nh.subscribe(sub);
  nh.advertise(Front_pub);
  nh.advertise(Right_pub);
  nh.advertise(Left_pub);
}

long publisher_timer;

void loop()
{

  int distancecenter = ultrasonic(echoPinFront, trigPinFront);
  msg.data = distancecenter;
  Front_pub.publish( &msg);

  int distanceright = ultrasonic(echoPinRight, trigPinRight);
  Right_msg.data = distanceright;
  Right_pub.publish( &Right_msg);

  int distanceleft = ultrasonic(echoPinLeft, trigPinLeft);
  Left_msg.data = distanceleft;
  Left_pub.publish( &Left_msg);


  MotorL(w_l * 10);
  MotorR(w_r * 10);
  nh.spinOnce();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers  String AX = String(mpu6050.getAccX());

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  String AX = String(AcX);
  String AY = String(AcY);
  String AZ = String(AcZ);
  String GX = String(GyX);
  String GY = String(GyY);
  String GZ = String(GyZ);
  String tmp = String(Tmp);

  String data = "A" + AX + "B" + AY + "C" + AZ + "D" + GX + "E" + GY + "F" + GZ + "G" ;
  Serial.println(data);
  int length = data.indexOf("G") + 2;
  char data_final[length + 1];
  data.toCharArray(data_final, length + 1);

  if (millis() > publisher_timer) {
    // step 1: request reading from sensor
    imu_msg.data = data_final;
    imu.publish(&imu_msg);
    publisher_timer = millis() + 100; //publish ten times a second
    nh.spinOnce();
  }
}

void Motors_init() {

  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}

void MotorR(int Pulse_Width2) {

  if (Pulse_Width2 > 0) {
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }
  if (Pulse_Width2 < 0) {
    Pulse_Width2 = abs(Pulse_Width2);
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }
  if (Pulse_Width2 == 0) {
    analogWrite(EN_R, Pulse_Width2);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }
}

void MotorL(int Pulse_Width1) {
  if (Pulse_Width1 > 0) {
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }
  if (Pulse_Width1 < 0) {
    Pulse_Width1 = abs(Pulse_Width1);
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }
  if (Pulse_Width1 == 0) {
    analogWrite(EN_L, Pulse_Width1);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
}


int ultrasonic(int echoPin, int trigPin) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}
