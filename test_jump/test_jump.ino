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
struct can_frame canMsg;
bool contact = false;
int count = 0;

MCP2515 mcp2515(SPI_CS_PIN);

void setup() {
  pinMode(INT_PIN, INPUT);
  while (!Serial);
  Serial.begin(921600);
  enterMotorMode(CALF_MOTOR_ID);
  enterMotorMode(THIGH_MOTOR_ID);
  delay(500);
}

void loop() {
  if (count < 500) {
    sendToMotor(CALF_MOTOR_ID, 0, 0, 0, 0, -4);
    count = count + 1;
  } else {
    sendToMotor(CALF_MOTOR_ID, 0, 0, 0, 0, 0);
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
