

# VCU Data Translation:

| Parameter               | Range                                  | Description                                                                                                                                                                        |
|-------------------------|----------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| nRightSpeedRef          | –30000 ~ 30000 rpm <br> –30000 ~ 30000 | (**_TODO_**: CLARIFY) Motor speed in revolutions per minute (rpm). 1 rpm steps.                                                                                                    |
| nLeftSpeedRef           | –30000 ~ 30000 rpm <br> –30000 ~ 30000 | (**_TODO_**: CLARIFY) Motor speed in revolutions per minute (rpm). 1 rpm steps.                                                                                                    |
| nControlMode            | 0, 1, 2, 3                             | 0 = STOP: vehicle stopped; <br> 1 = RCU: vehicle controlled by RCU; <br> 2 = GCU: vehicle controlled by GCU; <br> 3 = WAIT: waiting to receive commands to transition from 2 to 1; |


# Motor Drive Data Translation:

| Parameter               | Range                                  | Description                                                              |
|-------------------------|----------------------------------------|--------------------------------------------------------------------------|
| Controller_Temperature  | –100.0 ~ 300.0°C <br> –1000 ~ 3000     | The controller’s internal temperature (on the power base). 0.1°C steps.  |
| Motor_Temperature       | –100.0 ~ 300.0°C <br> –1000 ~ 3000     | Temperature sensor readout. 0.1°C steps.                         |
| Motor_RPM               | –30000 ~ 30000 rpm <br> –30000 ~ 30000 | Motor speed in revolutions per minute (rpm). 1 rpm steps.                |
| Battery_Current_Display | –3276.8 ~ 3276.7A <br> –32768 ~ 32767  | Calculated value in DC Amps. 0.1A steps.                                 |
