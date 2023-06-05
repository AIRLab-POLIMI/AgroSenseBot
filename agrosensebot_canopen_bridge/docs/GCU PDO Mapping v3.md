# Node IDs

- MDR (MDX): 0d38, 0x26
- MDL (MSX): 0d39, 0x27
- FAN: 0d40, 0x28
- VCU: 0d03, 0x03
- GCU: 0d05, 0x05


# GCU PDOs


## TPDO1 (to VCU)

COB-ID: `GCU_NODE_ID + 0x180 = 0x185`

Mapping:
```
    0x2111:01 TPDO1_1 (USINT, 1 byte)
        bit 0: bIsAlive
        bit 1: bReady
              TPDO1_2 (USINT, 1 byte) not used
```

Size: 2 bytes (1 byte?)


## TPDO2 (to VCU)

COB-ID: `GCU_NODE_ID + 0x280 = 0x285`

Mapping:
```
    0x2112:01 TPDO2_1 (INT, 2 bytes) nRightSpeedRef
    0x2112:02 TPDO2_2 (INT, 2 bytes) nLeftSpeedRef
```

Size: 4 bytes


## TPDO3 (to FAN)?
FAN speedRef?


## RPDO1 (from VCU)

COB-ID: `VCU_NODE_ID + ? = ?`

Mapping:
```
    0x2111:02 RPDO1_1 (USINT, 1 byte)
        bit 0 -> bVcuIsAlive
        bit 1 -> bSafetyStatus
    0x2111:03 RPDO1_2 (USINT, 1 byte)
        bit 0: nControlMode
```

Size: 2 bytes


## RPDO2 (from MDL)

COB-ID: `MDL_NODE_ID + 0x180 = 01A7`

Mapping:
```
    0x2110:01 RPDO2_1 (INT, 2 byte) Controller_Temperature
    0x2110:02 RPDO2_2 (INT, 2 byte) Motor_Temperature
    0x2110:03 RPDO2_3 (INT, 2 byte) Motor_RPM
    0x2110:04 RPDO2_4 (INT, 2 byte) Battery_Current_Display
```

Size: 8 bytes


## RPDO3 (from MDR)

COB-ID = `MDR_NODE_ID + 0x180 = 0x1A6`

Mapping:
```
    0x2110:05 RPDO3_1 (INT, 2 byte) Controller_Temperature
    0x2110:06 RPDO3_2 (INT, 2 byte) Motor_Temperature
    0x2110:07 RPDO3_3 (INT, 2 byte) Motor_RPM
    0x2110:08 RPDO3_4 (INT, 2 byte) Battery_Current_Display
```

Size: 8 bytes


## RPDO4 (from FAN):

COB-ID = `FAN_NODE_ID + 0x180 = 0x1A8`

Mapping:
```
    0x2110:09 RPDO4_1 (INT, 2 byte) Controller_Temperature
    0x2110:0A RPDO4_2 (INT, 2 byte) Motor_Temperature
    0x2110:0B RPDO4_3 (INT, 2 byte) Motor_RPM
    0x2110:0C RPDO4_4 (INT, 2 byte) Battery_Current_Display
```

Size: 8 bytes