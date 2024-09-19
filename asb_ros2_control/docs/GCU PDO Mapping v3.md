# Node IDs

| Node Name | ID (dec) | ID (hex) |
|-----------|----------|----------|
| VCU       | 0d03     | 0d03     |
| GCU       | 0d05     | 0x05     |
| RCU       | 0d10     | 0x0A     |
| MDR (MDX) | 0d38     | 0x26     |
| MDL (MSX) | 0d39     | 0x27     |
| FAN       | 0d40     | 0x28     |


# GCU PDOs

## TPDO1 (to VCU)

```
Defined in: 0x1800, 0x1A00
COB-ID:     GCU_NODE_ID + 0x180 = 0x185
```

Mapping:
```
TPDO1_1 (UINT8, 1 byte)
    bit 0: bIsAlive
    bit 1: not used
    bit 2: bPumpCmd
TPDO1_2 (UINT8, 1 byte) not used
```

Size: 2 bytes

## TPDO2 (to VCU)

```
Defined in: 0x1801, 0x1A01
COB-ID:     GCU_NODE_ID + 0x280 = 0x285
```

Mapping:
```
TPDO2_1 (INT16, 2 bytes) nRightSpeedRef
TPDO2_2 (INT16, 2 bytes) nLeftSpeedRef
TPDO2_2 (INT16, 2 bytes) nFanSpeedRef
```

Size: 4 bytes

## RPDO1 (from VCU)

```
Defined in: 0x1400, 0x1600
COB-ID:     GCU_NODE_ID + 0x200 = 0x205
```

Mapping:
```
RPDO1_1 (UINT8, 1 byte)
    bit 0: bVcuIsAlive
    bit 1: bSafetyStatus
    bit 2: bPumpStatus
RPDO1_2 (UINT8, 1 byte) nControlMode
RPDO1_3 (UINT8, 1 byte) nMoreRecentAlarmIdToConfirm
RPDO1_4 (UINT8, 1 byte) nMoreRecentActiveAlarmId
```

Size: 2 bytes

# Motor Drive PDOs

## RPDO1 (from MDL/MDR/FAN)

```
Defined in: 0x1400, 0x1600
COB-ID MDR: NODE_ID + 0x180 = 0x1A6
COB-ID MDL: NODE_ID + 0x180 = 0x1A7
COB-ID FAN: NODE_ID + 0x180 = 0x1A8
```

Mapping:
```
RPDO2_1 (INT16, 2 byte) Controller_Temperature
RPDO2_2 (INT16, 2 byte) Motor_Temperature
RPDO2_3 (INT16, 2 byte) Motor_RPM
RPDO2_4 (INT16, 2 byte) Battery_Current_Display
```

Size: 8 bytes

## RPDO2 (from MDL/MDR/FAN)

```
Defined in: 0x1401, 0x1601
COB-ID MDR: NODE_ID + 0x280 = 0x2A6
COB-ID MDL: NODE_ID + 0x280 = 0x2A7
COB-ID FAN: NODE_ID + 0x280 = 0x2A8
```

Mapping:
```
RPDO2_1 (INT16, 2 byte) Motor_Torque
RPDO2_2 (INT16, 2 byte) BDI_Percentage
RPDO2_3 (INT16, 2 byte) Keyswitch_Voltage
RPDO2_4 (INT16, 2 byte) Zero_Speed_Threshold
```

Size: 8 bytes

## RPDO3 (from MDL/MDR/FAN)

```
Defined in: 0x1402, 0x1602
COB-ID MDR: NODE_ID + 0x380 = 0x3A6
COB-ID MDL: NODE_ID + 0x380 = 0x3A7
COB-ID FAN: NODE_ID + 0x380 = 0x3A8
```

Mapping:
```
RPDO3_1 (UINT16, 2 byte)
    bit 0: bOutIsAlive (ignored by asb_ros2_control)
    bit 1: bInterlockStatus
    bit 2: bAlarm (ignored by asb_ros2_control)
    bit 3: bVcuNotActive (ignored by asb_ros2_control)
RPDO3_2 (UINT16, 2 byte) not used
RPDO3_3 (UINT16, 2 byte) not used
RPDO3_4 (UINT16, 2 byte) not used
```

Size: 8 bytes

## RPDO3 (from MDL/MDR/FAN)

```
Defined in: 0x1403, 0x1603
COB-ID MDR: NODE_ID + 0x480 = 0x4A6
COB-ID MDL: NODE_ID + 0x480 = 0x4A7
COB-ID FAN: NODE_ID + 0x480 = 0x4A8
```

Mapping:
```
RPDO4_1 (INT32, 4 byte) Rotor_Position
```

Size: 4 bytes
