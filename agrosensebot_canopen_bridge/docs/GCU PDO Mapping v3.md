# Node IDs

- MDR (MDX): 0d38, 0x26
- MDL (MSX): 0d39, 0x27
- FAN: 0d40, 0x28
- VCU: 0d03, 0x03
- GCU: 0d05, 0x05


# GCU PDOs


## TPDO1 (to VCU)

Defined in: `0x1800, 0x1A00`
COB-ID: `GCU_NODE_ID + 0x180 = 0x185`

Mapping:
```
    0x2111:01 TPDO1_1 (UINT8, 1 byte)
        bit 0: bIsAlive
        bit 1: bReady
              TPDO1_2 (UINT8, 1 byte) not used
```

Size: 2 bytes (1 byte?)


## TPDO2 (to VCU)

Defined in: `0x1801, 0x1A01`
COB-ID: `GCU_NODE_ID + 0x280 = 0x285`

Mapping:
```
    0x2112:01 TPDO2_1 (INT16, 2 bytes) nRightSpeedRef
    0x2112:02 TPDO2_2 (INT16, 2 bytes) nLeftSpeedRef
```

Size: 4 bytes


## TPDO3 (to FAN)?
FAN speedRef?


## RPDO1 (from VCU)

Defined in: `0x1400, 0x1600`
COB-ID: `GCU_NODE_ID + 0x200 = 0x205`

Mapping:
```
    0x2111:02 RPDO1_1 (UINT8, 1 byte)
        bit 0 -> bVcuIsAlive
        bit 1 -> bSafetyStatus
    0x2111:03 RPDO1_2 (UINT8, 1 byte)
        bit 0: nControlMode
```

Size: 2 bytes


## RPDO2 (from MDL)

Defined in: `0x1401, 0x1601`
COB-ID: `MDL_NODE_ID + 0x180 = 0x1A7`

Mapping:
```
    0x2110:01 RPDO2_1 (INT16, 2 byte) Controller_Temperature
    0x2110:02 RPDO2_2 (INT16, 2 byte) Motor_Temperature
    0x2110:03 RPDO2_3 (INT16, 2 byte) Motor_RPM
    0x2110:04 RPDO2_4 (INT16, 2 byte) Battery_Current_Display
```

Size: 8 bytes


## RPDO3 (from MDR)

Defined in: `0x1402, 0x1602`
COB-ID = `MDR_NODE_ID + 0x180 = 0x1A6`

Mapping:
```
    0x2110:05 RPDO3_1 (INT16, 2 byte) Controller_Temperature
    0x2110:06 RPDO3_2 (INT16, 2 byte) Motor_Temperature
    0x2110:07 RPDO3_3 (INT16, 2 byte) Motor_RPM
    0x2110:08 RPDO3_4 (INT16, 2 byte) Battery_Current_Display
```

Size: 8 bytes


## RPDO4 (from FAN):

Defined in: `0x1403, 0x1603`
COB-ID = `FAN_NODE_ID + 0x180 = 0x1A8`

Mapping:
```
    0x2110:09 RPDO4_1 (INT16, 2 byte) Controller_Temperature
    0x2110:0A RPDO4_2 (INT16, 2 byte) Motor_Temperature
    0x2110:0B RPDO4_3 (INT16, 2 byte) Motor_RPM
    0x2110:0C RPDO4_4 (INT16, 2 byte) Battery_Current_Display
```

Size: 8 bytes