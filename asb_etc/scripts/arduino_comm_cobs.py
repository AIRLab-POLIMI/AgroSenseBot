#!/usr/bin/python3
import sys
import serial
import struct
import time
from cobs import cobs  # smart binary serial encoding and decoding
from datetime import datetime
import pandas as pd

s = serial.Serial('/dev/ttyACM2', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=3)

COBS_DELIMITER = b'\x00'
PAYLOAD_SIZE_UINT32 = 3
PAYLOAD_SIZE_BYTES = 4 * PAYLOAD_SIZE_UINT32

t_prev = time.perf_counter()
error_count = 0

# saved data
t_list = list()
t_msg_list = list()
v_list = list()
p_list = list()

while True:
    try:
        encoded_data = s.read_until(COBS_DELIMITER)  # read until the COBS packet ending delimiter is found
        n = len(encoded_data)

        if n > 0:
            decoded_data = cobs.decode(encoded_data[0:-1])  # discard the delimiter byte and decode the data
            n_decoded = len(decoded_data)

            if n_decoded == PAYLOAD_SIZE_BYTES:
                t_now = time.perf_counter()
                t_elapsed = t_now - t_prev
                t_prev = t_now

                t, adc_value, delta_analog_read = struct.unpack('I' * PAYLOAD_SIZE_UINT32, decoded_data)  # python data types: https://docs.python.org/3/library/struct.html#format-characters
                sensor_value_voltage = adc_value / (2**12 - 1) * 5 # [V]
                sensor_value_pressure = (sensor_value_voltage - 0.5) / 4 * 1.2  # [MPa]

                t_msg = datetime.now()
                t_list.append(t)
                t_msg_list.append(t_msg)
                v_list.append(sensor_value_voltage)
                p_list.append(sensor_value_pressure)

                # print(f"t [µs] = {t}, value [MPa] = {sensor_value_pressure:+6.4f}, value [bar] = {sensor_value_pressure*10:+6.4f}, value [V] = {sensor_value_voltage:+6.4f}, analog_read [µs] = {delta_analog_read}, error_count = {error_count}")

            else:
                error_count += 1

    except KeyboardInterrupt as err:
        print("caught keyboard ctrl-c:".format(err))
        print("exiting.")
        break
    except (serial.serialutil.SerialException, cobs.DecodeError):
        print("Unexpected error:", sys.exc_info()[0])  # restart serial
        break

pd.DataFrame({'t_msg':t_msg_list, 't': t_list, 'v': v_list, 'p': p_list}).to_csv(f"/home/agrosensebot/tmp/p_{str(datetime.now()).replace(' ', '_')}.csv")
