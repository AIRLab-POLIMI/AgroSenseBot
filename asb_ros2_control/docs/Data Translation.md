Specification of the data values of the CANOpen motor drive, as per manual `Curtis Manual AC F2-T and AC F2-D – FOS 5.0 April 2023`.

### VCU Data Translation:

| Parameter      | Range                                  | Conversion Factor    | Description                                                                                                                                                                        |
|----------------|----------------------------------------|----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| nRightSpeedRef | –30000 ~ 30000 RPM <br> –30000 ~ 30000 |  1 RPM               | Motor speed in revolutions per minute (RPM).                                                                                                                                       |
| nLeftSpeedRef  | –30000 ~ 30000 RPM <br> –30000 ~ 30000 |  1 RPM               | Motor speed in revolutions per minute (RPM).                                                                                                                                       |
| nFanSpeedRef   | –30000 ~ 30000 RPM <br> –30000 ~ 30000 |  1 RPM               | Motor speed in revolutions per minute (RPM).                                                                                                                                       |
| nControlMode   | 0, 1, 2, 3                             |                      | 0 = STOP: vehicle stopped; <br> 1 = RCU: vehicle controlled by RCU; <br> 2 = GCU: vehicle controlled by GCU; <br> 3 = WAIT: waiting to receive commands to transition from 2 to 1; |


### Motor Drive Data Translation:

| Parameter               | Range                                                       | Conversion Factor | Description                                                                                                                  |
|-------------------------|-------------------------------------------------------------|-------------------|------------------------------------------------------------------------------------------------------------------------------|
| Controller_Temperature  | –100.0 ~ 300.0°C <br> –1000 ~ 3000                          | 0.1 °C            | The controller’s internal temperature (on the power base).                                                                   |
| Motor_Temperature       | –100.0 ~ 300.0°C <br> –1000 ~ 3000                          | 0.1 °C            | Temperature sensor readout.                                                                                                  |
| Motor_RPM               | –30000 ~ 30000 RPM <br> –30000 ~ 30000                      | 1 RPM             | Motor speed in revolutions per minute (RPM).                                                                                 |
| Battery_Current_Display | –3276.8 ~ 3276.7A <br> –32768 ~ 32767                       | 0.1 A             | Calculated value in DC Amps.                                                                                                 |
| Motor_Torque            | –500 ~ 500 Nm <br> –500 ~ 500                               | 1 Nm              | A calculated value in Nm. This is only available for (Curtis) dyno-characterized motors. Uncharacterized motors will read 0. |
| BDI_Percentage          | 0 ~ 100% <br> 0 ~ 100                                       | 1 %               | Battery state-of-charge (SOC).                                                                                               |
| Keyswitch_Voltage       | 0.00 ~ 105.00V <br> 0 ~ 10500                               | 0.01 V            | Voltage at KSI (pin 1).                                                                                                      |
| Zero_Speed_Threshold    | 5 ~ 300 RPM <br> 5 ~ 300                                    | 1 RPM             | Determines the speed below which the EM brake is set (On).                                                                   |
| Position_Rotor          | –524288.000 ~ 524288.000 revs <br> –2147483648 ~ 2147483647 | 2^-12 revs        | Rotor revolutions since the last KSI = On.                                                                                   |
