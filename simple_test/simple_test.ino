#include <SPI.h>
#include <mcp2515.h>

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

const int SPI_CS_PIN = 10;
const int INT_PIN = 2;

const int motor_1_id = 1;
const int motor_2_id = 2;

const bool motor_1_open = true;
const bool motor_2_open = true;

struct can_frame canMsg;
struct can_frame turnOff;

MCP2515 mcp2515(SPI_CS_PIN);

void setup() {
  pinMode(INT_PIN, INPUT);
  while (!Serial);
  Serial.begin(115200);
  //  Serial.begin(921600);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  enterMotorMode(motor_1_id);
  enterMotorMode(motor_2_id);

  delay(1000);
}

void loop() {
  sendToMotor(motor_2_id, 30 * 3.14 / 180, 0, 10, 1, 0);
  sendToMotor(motor_1_id, 30 * 3.14 / 180, 0, 3, 1, 0);
  delay(5);
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


//Function by Ben Katz:
//https://os.mbed.com/users/benkatz/code/CanMasterTest//file/d24fd64d1fcb/math_ops.cpp/
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
