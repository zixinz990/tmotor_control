// -------------------------------------------------------------
// CANtest for Teensy 3.6 dual CAN bus
// by Collin Kidder, Based on CANTest by Pawelsky (based on CANtest by teachop)
//
// Both buses are left at default 250k speed and the second bus sends frames to the first
// to do this properly you should have the two buses linked together. This sketch
// also assumes that you need to set enable pins active. Comment out if not using
// enable pins or set them to your correct pins.
//
// This sketch tests both buses as well as interrupt driven Rx and Tx. There are only
// two Tx buffers by default so sending 5 at a time forces the interrupt driven system
// to buffer the final three and send them via interrupts. All the while all Rx frames
// are internally saved to a software buffer by the interrupt handler.
//

#include <FlexCAN_T4.h>

/*#ifndef __MK20DX128__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
  #endif*/

static CAN_message_t msg;
static uint8_t hex[17] = "0123456789abcdef";

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}

void setup(void)
{
  delay(1000);
  Serial.println(F("Hello Teensy 4.1 dual CAN Test."));

  Can0.begin();

  pinMode(2, OUTPUT);
  pinMode(35, OUTPUT);

  digitalWrite(2, HIGH);
  digitalWrite(35, HIGH);
  // Enter the 8-Byte Hex Data
  msg.flags.extended = 0;
  msg.id = 0x001;
  msg.len = 8;
  msg.buf[0] = 0x8d;
  msg.buf[1] = 0x66;
  msg.buf[2] = 0x7f;
  msg.buf[3] = 0xf0;
  msg.buf[4] = 0x10;
  msg.buf[5] = 0x33;
  msg.buf[6] = 0x37;
  msg.buf[7] = 0xff;

  CAN_message_t intl;
  // Enter M_Mode which is motor control mode(don't need to be change)
  intl.flags.extended = 0;
  intl.id = 0x001;
  intl.len = 8;
  intl.buf[0] = 0xFF;
  intl.buf[1] = 0xFF;
  intl.buf[2] = 0xFF;
  intl.buf[3] = 0xFF;
  intl.buf[4] = 0xFF;
  intl.buf[5] = 0xFF;
  intl.buf[6] = 0xFF;
  intl.buf[7] = 0xFC;
  Can0.write(intl);

  // Set the position to 0 when it do the position mode(don't need to be change)
  intl.flags.extended = 0;
  intl.id = 0x001;
  intl.len = 8;
  intl.buf[0] = 0xFF;
  intl.buf[1] = 0xFF;
  intl.buf[2] = 0xFF;
  intl.buf[3] = 0xFF;
  intl.buf[4] = 0xFF;
  intl.buf[5] = 0xFF;
  intl.buf[6] = 0xFF;
  intl.buf[7] = 0xFE;
  Can0.write(intl);
}

void loop(void)
{
  //CAN_message_t inMsg;
  /* while (Can0.available())
    {
     Can0.read(inMsg);
     Serial.print("CAN bus 0: "); hexDump(8, inMsg.buf);
    }*/
  // msg.buf[0]++;
  //Can0.write(msg);
  /*  msg.buf[0]++;
    Can0.write(msg);
    msg.buf[0]++;
    Can0.write(msg);
    msg.buf[0]++;
    Can0.write(msg);
    msg.buf[0]++;
    Can0.write(msg);  */
  //delay(200000);
}
