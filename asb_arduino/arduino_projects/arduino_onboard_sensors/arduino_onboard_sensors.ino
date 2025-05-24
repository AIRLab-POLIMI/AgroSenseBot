#include <PacketSerial.h>
PacketSerial myPacketSerial;

#define PAYLOAD_SIZE_UINT32 3

uint32_t data_uint32[PAYLOAD_SIZE_UINT32];
uint8_t data_payload[PAYLOAD_SIZE_UINT32 * 4];

void setup() {
  analogReadResolution(12);

  myPacketSerial.begin(115200);
}

void loop() {
  
  uint32_t t_now = micros();
  uint32_t sensor_value = analogRead(A0);
  uint32_t t_analog_read = micros() - t_now;
  
  data_uint32[0] = t_now;
  data_uint32[1] = sensor_value;
  data_uint32[2] = t_analog_read;
  
  memcpy(data_payload, data_uint32, sizeof(data_uint32));
  myPacketSerial.send(data_payload, sizeof(data_payload));
  
  delayMicroseconds(500);
}
