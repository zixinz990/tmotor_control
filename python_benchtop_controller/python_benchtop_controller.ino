#include <SPI.h>
#include <mcp2515.h>

#define P_MIN  -12.5f
#define P_MAX  12.5f
#define V_MIN  -45.0f
#define V_MAX  45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN  -10.0f
#define T_MAX  10.0f

const int SPI_CS_PIN = 10;
const int INT_PIN = 2;

const int motor_1_id = 0x01;
const int motor_2_id = 0x02;

const bool motor_1_open = true;
const bool motor_2_open = false;

struct can_frame canMsg1;
struct can_frame canMsg2;

MCP2515 mcp2515(SPI_CS_PIN);

String command;
float cmd_to_motors[5]; // position, velocity, kp, kd, torque

void setup() {
  pinMode(INT_PIN, INPUT);
  
  while (!Serial);
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();

  if (motor_1_open) {
    motorZero(motor_1_id);
    motorMode(motor_1_id);
    Serial.print("Enter motor mode");
  }

  if (motor_2_open) {
    motorZero(motor_2_id);
    motorMode(motor_2_id);
  }
}

void loop() {
  char buf[50];
  if (Serial.available() > 0) {
    command = Serial.readString();
    command.trim();
    command.toCharArray(buf, 50);
    char *i;
    cmd_to_motors[0] = atof(strtok_r(buf, ",", &i));
    for (int a = 0; a < 4; a++) {
      cmd_to_motors[a + 1] = atof(strtok_r(NULL, ",", &i));
    }
  }

  if (motor_1_open) {
    sendToMotor(motor_1_id, cmd_to_motors[0], -cmd_to_motors[1], cmd_to_motors[2], cmd_to_motors[3], cmd_to_motors[4]);
    bool test = mcp2515.readMessage(&canMsg1) == MCP2515::ERROR_OK;
    
    Serial.println(mcp2515.readMessage(&canMsg1));
    
    if (mcp2515.readMessage(&canMsg1) == MCP2515::ERROR_OK) {
      Serial.print(canMsg1.data[0], HEX);
      Serial.print(" ");
      Serial.print(uint_to_float((canMsg1.data[1] << 8) | canMsg1.data[2], P_MIN, P_MAX, 16));
      Serial.print(",");
      Serial.print(uint_to_float((canMsg1.data[3] << 4) | (canMsg1.data[4] >> 4), V_MIN, V_MAX, 12));
      Serial.print(",");
      Serial.print(uint_to_float(((canMsg1.data[4] & 0xF) << 8) | canMsg1.data[5], P_MIN, P_MAX, 12));
      Serial.println();
    }
  }

  if (motor_2_open) {
    sendToMotor(motor_2_id, cmd_to_motors[0], cmd_to_motors[1], cmd_to_motors[2], cmd_to_motors[3], cmd_to_motors[4]);
    if (mcp2515.readMessage(&canMsg2) == MCP2515::ERROR_OK) {
      Serial.print(canMsg2.data[0], HEX);
      Serial.print(" ");
      Serial.print(uint_to_float((canMsg2.data[1] << 8) | canMsg2.data[2], P_MIN, P_MAX, 16));
      Serial.print(",");
      Serial.print(uint_to_float((canMsg2.data[3] << 4) | (canMsg2.data[4] >> 4), V_MIN, V_MAX, 12));
      Serial.print(",");
      Serial.print(uint_to_float(((canMsg2.data[4] & 0xF) << 8) | canMsg2.data[5], P_MIN, P_MAX, 12));
      Serial.println();
    }
  }
  
  delay(25);
}

void sendToMotor(int mot_id, float pos, float vel, float kp, float kd, float tau) {
  struct can_frame cf;

  unsigned int con_pos = float_to_uint(constrain(pos, P_MIN, P_MAX), P_MIN, P_MAX, 16);
  unsigned int con_vel = float_to_uint(constrain(vel, V_MIN, V_MAX), V_MIN, V_MAX, 12);
  unsigned int con_kp = float_to_uint(constrain(kp, KP_MIN, KP_MAX), KP_MIN, KP_MAX, 12);
  unsigned int con_kd = float_to_uint(constrain(kd, KD_MIN, KD_MAX), KD_MIN, KD_MAX, 12);
  unsigned int con_tau = float_to_uint(constrain(tau, T_MIN, T_MAX), T_MIN, T_MAX, 12);

  cf.can_id  = mot_id;
  cf.can_dlc = 8;
  cf.data[0] = con_pos >> 8;
  cf.data[1] = con_pos & 0xFF;
  cf.data[2] = con_vel >> 4;
  cf.data[3] = ((con_vel & 0xF) << 4) | (con_kp >> 8);
  cf.data[4] = con_kp & 0xFF;
  cf.data[5] = con_kd >> 4;
  cf.data[6] = ((con_kd & 0xF) << 4) | (con_tau >> 8);
  cf.data[7] = con_tau & 0xFF;

  mcp2515.sendMessage(&cf);
}

void motorMode(int mot_id) {
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

void motorZero(int mot_id) {
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
  cf.data[7] = 0xFE;

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

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float)x_int) * span / 4095.0 + offset;
  } else if (bits == 16) {
    pgg = ((float)x_int) * span / 65535.0 + offset;
  }
  return pgg;
}
