# packet structure

## pmd -> pc

big-endian 64 bytes total

actual_position: int32[4]
actual_velocity: int32[4]
actual_current: int32[4]
actual_state: int32[4]

## pc -> pmd

big-endian 64 bytes total

set_position: int32[4]
set_velocity: int32[4]
set_current: int32[4]
set_state: int32[4]

## state:

0b0000 0000: motor off
0x0000 0111: current control
0x0001 0111: position control
0x0011 0111: trajectory control

