#include <SPI.h>
#include <JY901.h>
#include <Wire.h>
#include <mcp2515.h>

// Constants
#define FORCE_SENSOR_PIN A0
#define SPI_CS_PIN 10
#define INT_PIN 2

#define CALF_MOTOR_ID 1
#define THIGH_MOTOR_ID 2

#define DEFAULT_CALF_POS 0
#define DEFAULT_THIGH_POS 0

const float sample_rate = 200;
const float delta_t = 5;

// Global variables
// Motors
struct can_frame canMsg;

// Pressue sensor
bool contact = false;

// IMU
float lin_vel_x = 0;
float lin_vel_y = 0;
float lin_vel_z = 0;

float pos_x = 0;
float pos_y = 0;
float pos_z = 0;

// Low-pass filter
const float alpha = 0.5;
float filtered_lin_acc_x = 0;
float filtered_lin_acc_y = 0;
float filtered_lin_acc_z = 0;

MCP2515 mcp2515(SPI_CS_PIN);
CJY901 IMU_50 = CJY901();

void setup() {
  pinMode(INT_PIN, INPUT);
  while (!Serial);
  Serial.begin(921600);
  IMU_50.StartIIC(0x50);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  enterMotorMode(CALF_MOTOR_ID);
  enterMotorMode(THIGH_MOTOR_ID);
  delay(3000);
}

void loop() {
  // Check contact
  int analogReading = analogRead(FORCE_SENSOR_PIN);
  contact = analogReading >= 10;

  // Update velocity estimation
  IMU_50.GetAcc();

  float lin_acc_x = static_cast<float>(IMU_50.stcAcc.a[0]) / 32768 * 16 * 9.8;
  float lin_acc_y = static_cast<float>(IMU_50.stcAcc.a[1]) / 32768 * 16 * 9.8;
  float lin_acc_z = static_cast<float>(IMU_50.stcAcc.a[2]) / 32768 * 16 * 9.8;

  filtered_lin_acc_x = alpha * filtered_lin_acc_x + (1.0 - alpha) * lin_acc_x;
  filtered_lin_acc_y = alpha * filtered_lin_acc_y + (1.0 - alpha) * lin_acc_y;
  filtered_lin_acc_z = alpha * filtered_lin_acc_z + (1.0 - alpha) * lin_acc_z;

  lin_vel_x += filtered_lin_acc_x * delta_t;
  lin_vel_y += filtered_lin_acc_y * delta_t;
  lin_vel_z += filtered_lin_acc_z * delta_t;

  pos_x += lin_vel_x * delta_t;
  pos_y += lin_vel_y * delta_t;
  pos_z += lin_vel_z * delta_t;

  // Control loop
  if (!contact) {
    // Control joint positions if contact == false
    sendToMotor(motor_2_id, 30 * 3.14 / 180, 0, 10, 1, 0);  // kp & kd need tuning
    sendToMotor(motor_1_id, 30 * 3.14 / 180, 0, 3, 1, 0);   // kp & kd need tuning
  } else {
    // Control torque if contact == true
    if (lin_vel_y < 0) {
      // Give a very small torque command, if velocity < 0
      sendToMotor(CALF_MOTOR_ID, 0, 0, 0, 0, -2);  // torque command need tuning
      sendToMotor(THIGH_MOTOR_ID, 0, 0, 0, 0, 0);
    } else {
      // Jump, if velocity >= 0
      sendToMotor(CALF_MOTOR_ID, 0, 0, 0, 0, -10);  // torque command need tuning
      sendToMotor(THIGH_MOTOR_ID, 0, 0, 0, 0, 0);
    }
  }
  delay(1000 / sample_rate);
}

void enterMotorMode(int mot_id) {
  struct can_frame cf;
  cf.can_id  = mot_id;
  cf.can_dlc = 8;
  cf.data[0] = 0xFF;
  cf.data[1] = 0xFF;
  cf.data[2] = 0xFF;
  cf.data[3] = 0xFF;
  cf.data[4] = 0xFF;
  cf.data[5] = 0xFF;
  cf.data[6] = 0xFF;
  cf.data[7] = 0xFC;
  mcp2515.sendMessage(&cf);
}

void sendToMotor(int mot_id, float pos, float vel, float kp, float kd, float torq) {
  struct can_frame cf;
  unsigned int con_pos = float_to_uint(constrain(pos, P_MIN, P_MAX), P_MIN, P_MAX, 16);
  unsigned int con_vel = float_to_uint(constrain(vel, V_MIN, V_MAX), V_MIN, V_MAX, 12);
  unsigned int con_kp = float_to_uint(constrain(kp, KP_MIN, KP_MAX), KP_MIN, KP_MAX, 12);
  unsigned int con_kd = float_to_uint(constrain(kd, KD_MIN, KD_MAX), KD_MIN, KD_MAX, 12);
  unsigned int con_torq = float_to_uint(constrain(torq, T_MIN, T_MAX), T_MIN, T_MAX, 12);
  cf.can_id  = mot_id;
  cf.can_dlc = 8;
  cf.data[0] = con_pos >> 8;
  cf.data[1] = con_pos & 0xFF;
  cf.data[2] = con_vel >> 4;
  cf.data[3] = ((con_vel & 0xF) << 4) | (con_kp >> 8);
  cf.data[4] = con_kp & 0xFF;
  cf.data[5] = con_kd >> 4;
  cf.data[6] = ((con_kd & 0xF) << 4) | (con_torq >> 8);
  cf.data[7] = con_torq & 0xFF;
  mcp2515.sendMessage(&cf);
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
  // Converts a float to an unsigned int, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int) ((x - offset) * 4095.0 / span);
  } else if (bits == 16) {
    pgg = (unsigned int) ((x - offset) * 65535.0 / span);
  }
  return pgg;
}
