/*
    Description: Firmware for m5bot
*/
#define M5STACK_MPU6886

#include <Arduino.h>
#include <M5Stack.h>
#include <Wire.h>
#include <WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "wifi_setting.h"
#include "wifi_hardware.h"

// Servo Driver Settings
#define SERVO_ADDR 0x53

// Size Parameters
#define DISTANCE_LEFT_TO_RIGHT_WHEEL 0.105 // [m]
#define DISTANCE_FRONT_TO_REAR_WHEEL 0.095 // [m]
#define WHEEL_SEPARATION_WIDTH DISTANCE_LEFT_TO_RIGHT_WHEEL / 2
#define WHEEL_SEPARATION_LENGTH DISTANCE_FRONT_TO_REAR_WHEEL / 2
#define WHEEL_RADIUS 0.032 // [m]

// Servo Parameters
#define MAX_ANGULAR_VEL 8.72665  // [rad/sec]
#define MIN_PULSE 500            // [us]
#define MAX_PULSE 2500           // [us]
#define NEUTRAL_POSITION_PULSE (MIN_PULSE + (MAX_PULSE - MIN_PULSE) / 2)
#define PULSE_RANGE (MAX_PULSE - MIN_PULSE)

#define SPIN_PERIOD 100

#define CMD_VEL_TOPIC "/m5bot/cmd_vel"
#define IMU_TOPIC "/m5bot/imu"

struct MOTOR_VEL {
    int16_t v0; // front_left
    int16_t v1; // front_right
    int16_t v2; // rear_left
    int16_t v3; // rear_right
};

/*
    ROS Node Settings
*/
// Node Handler
ros::NodeHandle_<WiFiHardware> nh;
// Create Subscriber & Subscribe Callback
// Subscribing CMD_VEL_TOPIC
void messageCb(const geometry_msgs::Twist& twist) {
    setVelocity(twistToMotorVel(twist));
}
ros::Subscriber<geometry_msgs::Twist> sub(CMD_VEL_TOPIC, &messageCb);
// Create Publisher
// Publishing IMU_TOPIC
sensor_msgs::Imu imu_msg;
ros::Publisher pub(IMU_TOPIC, &imu_msg);

/*
    Connect To WiFi AP
*/
void setupWiFi() {
    // Establish WiFi Connection
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
    }
    // Show connection info on the screen
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("WiFi connected!");
    M5.Lcd.printf("IP address: ");
    M5.Lcd.println(WiFi.localIP());
}

/*
    Initialization
*/
void setup() {
    // Initialize M5 Board
    M5.begin(true, false, true);
    M5.Power.begin();
    M5.IMU.Init();
    Wire.begin(21, 22, 100000);

    // Connect To WiFi AP
    setupWiFi();

    // Initialize ROS Node
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);

    // put your setup code here, to run once:
}

/*
    Write us value to Servo Motor
*/
void Servo_write_us(uint8_t number, uint16_t us) {
    Wire.beginTransmission(SERVO_ADDR);
    Wire.write(0x00 | number);
    Wire.write(us & 0x00ff);
    Wire.write(us >> 8 & 0x00ff);
    Wire.endTransmission();
}

/*
    Check MOTOR_VEL Value & Clip value to the MAX&MIN range
*/
void checkLimit(struct MOTOR_VEL *command) {
    if (command->v0 > MAX_PULSE) command->v0 = MAX_PULSE;
    if (command->v0 < MIN_PULSE) command->v0 = MIN_PULSE;
    if (command->v1 > MAX_PULSE) command->v1 = MAX_PULSE;
    if (command->v1 < MIN_PULSE) command->v1 = MIN_PULSE;
    if (command->v2 > MAX_PULSE) command->v2 = MAX_PULSE;
    if (command->v2 < MIN_PULSE) command->v2 = MIN_PULSE;
    if (command->v3 > MAX_PULSE) command->v3 = MAX_PULSE;
    if (command->v3 < MIN_PULSE) command->v3 = MIN_PULSE;
}

/*
    Convert twist to Motor Velocity
*/
MOTOR_VEL twistToMotorVel(geometry_msgs::Twist twist) {
    MOTOR_VEL command;
    // Calculate Motor Velocity with Inverse Kinematics
    command.v0 = convertToUS((1/WHEEL_RADIUS) * (twist.linear.x - twist.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * twist.angular.z));
    command.v1 = convertToUS((1/WHEEL_RADIUS) * (twist.linear.x + twist.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * twist.angular.z) * (-1));
    command.v2 = convertToUS((1/WHEEL_RADIUS) * (twist.linear.x + twist.linear.y - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * twist.angular.z));
    command.v3 = convertToUS((1/WHEEL_RADIUS) * (twist.linear.x - twist.linear.y + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) * twist.angular.z) * (-1));

    checkLimit(&command);
    return command;
}

/*
    Convert rad/sec to pulse value
*/
int16_t convertToUS(float ref_angular_vel) {
    int16_t us = (int16_t)(PULSE_RANGE/2/MAX_ANGULAR_VEL*ref_angular_vel + NEUTRAL_POSITION_PULSE);
    return us;
}

/*
    Set Velocity
*/
void setVelocity(MOTOR_VEL command) {
    Servo_write_us(0, command.v0);
    Servo_write_us(1, command.v1);
    Servo_write_us(2, command.v2);
    Servo_write_us(3, command.v3);
}

/*
    Get IMU Data from M5Go
*/
sensor_msgs::Imu getImu() {
    // Get IMU Data
    sensor_msgs::Imu imu;
    float accX = 0.0F;
    float accY = 0.0F;
    float accZ = 0.0F;
    float gyroX = 0.0F;
    float gyroY = 0.0F;
    float gyroZ = 0.0F;
    float pitch = 0.0F;
    float roll = 0.0F;
    float yaw = 0.0F;
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    M5.IMU.getAccelData(&accX, &accY, &accZ);
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    imu.angular_velocity.x = gyroX;
    imu.angular_velocity.y = gyroY;
    imu.angular_velocity.z = gyroZ;
    imu.linear_acceleration.x = accX;
    imu.linear_acceleration.y = accY;
    imu.linear_acceleration.z = accZ;
    imu.orientation.x = roll;
    imu.orientation.y = pitch;
    imu.orientation.z = yaw;
    imu.orientation.w = 1;

    // Show on the screen
    M5.Lcd.setCursor(0, 65);
    M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
    M5.Lcd.setCursor(220, 87);
    M5.Lcd.print(" o/s");
    M5.Lcd.setCursor(0, 110);
    M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
    M5.Lcd.setCursor(220, 132);
    M5.Lcd.print(" G");
    M5.Lcd.setCursor(0, 154);
    M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);
    M5.Lcd.setCursor(220, 176);
    M5.Lcd.print(" degree");

    return imu;
}

/*
    Main Loop
*/
void loop() {
    imu_msg = getImu();
    pub.publish(&imu_msg);
    nh.spinOnce();
    delay(SPIN_PERIOD);
}