#include <JY901.h>
#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

float delta_t = 5;  // ms

CJY901 IMU_50 = CJY901();

float lin_vel_x = 0;
float lin_vel_y = 0;
float lin_vel_z = 0;

float pos_x = 0;
float pos_y = 0;
float pos_z = 0;

// Low-pass filter
const float alpha = 0.5;  // set alpha = 0 to close the filter
float filtered_lin_acc_x = 0;
float filtered_lin_acc_y = 0;
float filtered_lin_acc_z = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(921600);
  IMU_50.StartIIC(0x50);
}

void loop() {
  // Get lin acc data
  IMU_50.GetAcc();
  lin_acc_x = (float)IMU_50.stcAcc.a[0] / 32768 * 16 * 9.8 - 9.8;
  lin_acc_y = (float)IMU_50.stcAcc.a[1] / 32768 * 16 * 9.8 - 9.8;
  lin_acc_z = (float)IMU_50.stcAcc.a[2] / 32768 * 16 * 9.8 - 9.8;

  // Filter lin acc data
  filtered_lin_acc_x = alpha * filtered_lin_acc_x + (1.0 - alpha) * lin_acc_x;
  filtered_lin_acc_y = alpha * filtered_lin_acc_y + (1.0 - alpha) * lin_acc_y;
  filtered_lin_acc_z = alpha * filtered_lin_acc_z + (1.0 - alpha) * lin_acc_z;

  // Update velocity estimation
  // @todo: subtract gravity acc
  lin_vel_x += filtered_lin_acc_x * delta_t;
  lin_vel_y += filtered_lin_acc_y * delta_t;
  lin_vel_z += filtered_lin_acc_z * delta_t;

  // Update position estimation
  pos_x += lin_vel_x * delta_t;
  pos_y += lin_vel_y * delta_t;
  pos_z += lin_vel_z * delta_t;

  delay(delta_t);
}
