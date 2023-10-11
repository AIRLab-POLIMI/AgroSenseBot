builtin_interfaces/Time stamp

bool vcu_comm_ok
bool vcu_safety_status
uint8 control_mode  # 0 = STOP: vehicle stopped;
                    # 1 = RCU: vehicle controlled by RCU;
                    # 2 = GCU: vehicle controlled by GCU;
                    # 3 = WAIT: waiting to receive commands to transition from 2 to 1;

bool software_emergency_stop

bool pump_state

float64 left_motor_controller_temperature  # °C
float64 left_motor_temperature  # °C
float64 left_motor_battery_current  # A
float64 left_motor_torque  # Nm
float64 left_motor_bdi_percentage  # %, battery state of charge
float64 left_motor_keyswitch_voltage  # V, battery voltage
int64 left_motor_zero_speed_threshold  # RPM, the speed below which the EM brake is set

float64 left_motor_position  # rad
float64 left_motor_velocity  # rad/s

float64 right_motor_controller_temperature  # °C
float64 right_motor_temperature  # °C
float64 right_motor_battery_current  # A
float64 right_motor_torque  # Nm
float64 right_motor_bdi_percentage  # %, battery state of charge
float64 right_motor_keyswitch_voltage  # V, battery voltage
int64 right_motor_zero_speed_threshold  # RPM, the speed below which the EM brake is set

float64 right_motor_position  # rad
float64 right_motor_velocity  # rad/s

float64 fan_motor_controller_temperature  # °C
float64 fan_motor_temperature  # °C
float64 fan_motor_battery_current  # A
float64 fan_motor_torque  # Nm
float64 fan_motor_bdi_percentage  # %, battery state of charge
float64 fan_motor_keyswitch_voltage  # V, battery voltage
int64 fan_motor_zero_speed_threshold  # RPM, the speed below which the EM brake is set

float64 fan_motor_position  # revolutions
float64 fan_motor_velocity_rpm  # RPM
