#include <PacketSerial.h>
#include "asb_BME280.h"
#include <Wire.h>

PacketSerial myPacketSerial;
BME280 bme280;

#define DEBUG_SERIAL_PRINT 0

typedef struct sensorData_t {
  uint32_t analog_A0_value;
  float ambient_pressure;
  float ambient_temperature;
  float ambient_relative_humidity;
};

typedef union dataPayload_t {
  sensorData_t sensor_data;
  byte sensor_data_bytes[sizeof(sensorData_t)];
};
dataPayload_t data_payload;

void setup() {
  analogReadResolution(12);

#if DEBUG_SERIAL_PRINT == 1

  Serial.begin(115200);
  if (!bme280.init()) {
    Serial.println("Device error!");
  }

#else

  myPacketSerial.begin(115200);
  bme280.init();

#endif

}

void loop() {

  uint32_t t_analog_read_start = micros();
  uint32_t analog_A0_value = analogRead(A0);
  uint32_t t_analog_read = micros() - t_analog_read_start;

  uint32_t t_bme280_request_data_start = micros();
  bme280.requestData();
  uint32_t t_bme280_request_data = micros() - t_bme280_request_data_start;

#if DEBUG_SERIAL_PRINT == 1

  Serial.print("   A0: ");
  Serial.print(analog_A0_value);

  Serial.print("   P: ");
  Serial.print(bme280.last_pressure);

  Serial.print("   T: ");
  Serial.print(bme280.last_temperature);

  Serial.print("   H: ");
  Serial.print(bme280.last_humidity);

  Serial.print("   t analog_read: ");
  Serial.print(t_analog_read);

  Serial.print("   t bme280: ");
  Serial.print(t_bme280_request_data);

  Serial.print("   burst_transmission: ");
  Serial.print(bme280.burst_transmission_micros);

  Serial.print("   burst_response: ");
  Serial.print(bme280.burst_response_micros);

  Serial.print("   burst_read: ");
  Serial.print(bme280.burst_read_micros);

  Serial.println(" ");

#else

  data_payload.sensor_data.analog_A0_value = analog_A0_value;
  data_payload.sensor_data.ambient_pressure = bme280.last_pressure;
  data_payload.sensor_data.ambient_temperature = bme280.last_temperature;
  data_payload.sensor_data.ambient_relative_humidity = bme280.last_humidity;

  myPacketSerial.send(data_payload.sensor_data_bytes, sizeof(data_payload.sensor_data_bytes));

#endif
}
