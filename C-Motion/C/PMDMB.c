//  pmdmb.c -- MotionBoard specific API
//
//  Performance Motion Devices, Inc.
//

#include "pmdmb.h"


#define MB_USER_IO      0x00
#define MB_READ_USER_O  0x00
#define MB_WRITE_USER_O 0x00
#define MB_AMP_ENABLE   0x01
#define MB_RESET_CAUSE  0x02
#define MB_SERIAL_PORT  0x03
#define MB_WATCHDOG     0x04
#define MB_PLD          0xFF
#define MB_SSI_RESET    0x1F
#define MB_SSI_VERSION  0x1F
#define MB_SSI_POSITION 0x10
#define MB_SSI_CLOCK    0x11



//*****************************************************************************
PMDuint16 PMDMBReadDigitalInput(PMDAxisHandle* axis_handle, PMDuint16* state)
{
    PMDuint16 rc;
    PMDuint16 value;

    // the upper 8 bits reflect the status of the 8 user input signals
    rc = PMDReadIO(axis_handle, MB_USER_IO, &value);
    value >>= 8;
    *state = value;

    return rc;
}

//*****************************************************************************
PMDuint16 PMDMBReadDigitalOutput(PMDAxisHandle* axis_handle, PMDuint16* state)
{
    PMDuint16 rc;
    PMDuint16 value;

    // the lower 8 bits reflect the status of the 8 user output signals
    rc = PMDReadIO(axis_handle, MB_READ_USER_O, &value);
    value &= 0x00FF;
    *state = value;

    return rc;
}

//*****************************************************************************
PMDuint16 PMDMBWriteDigitalOutput(PMDAxisHandle* axis_handle, PMDuint16 state)
{
    return PMDWriteIO(axis_handle, MB_WRITE_USER_O, state);
}

/*****************************************************************************
    // axismask: 1=X, 2=Y, 4=Z, 8=W
    // state: 1=X, 2=Y, 4=Z, 8=W
    // to change to the corresponding bit in the state (bits [0..3])
    // the axismask is the change mask. Any bit that is a 1 in the axismask will
    // cause the corresponding bit in the state to take effect.
    // hence an axismask of 0x000F will force all bits in the state to take effect
******************************************************************************/
PMDuint16 PMDMBSetAmplifierEnable(PMDAxisHandle* axis_handle, PMDuint16 axismask, PMDuint16 state)
{
    axismask &= 0x000F;
    axismask <<= 8;
    state |= axismask;
    // bits [0..3] are the amp enable bits for each axis
    // bits [8..11] are the change mask bits for each axis
    
    return PMDWriteIO(axis_handle, MB_AMP_ENABLE, state);
}

//*****************************************************************************
PMDuint16 PMDMBGetAmplifierEnable(PMDAxisHandle* axis_handle, PMDuint16* state)
{
    PMDuint16 rc;
    PMDuint16 value;

    rc = PMDReadIO(axis_handle, MB_AMP_ENABLE, &value);
    value &= 0x000F;
    *state = value;

    return rc;
}

/*****************************************************************************
    DAC bit is bit 7 in the amp enable register
******************************************************************************/
PMDuint16 PMDMBSetDACOutputEnable(PMDAxisHandle* axis_handle, PMDuint16 state)
{
    state &= 0x0001; // clear any stray bits
    state <<= 7;     // DAC enable bit is bit 7
    state |= 0x8000; // set the change bit
    
    return PMDWriteIO(axis_handle, MB_AMP_ENABLE, state);
}

/*****************************************************************************
    DAC bit is bit 7 in the amp enable register
******************************************************************************/
PMDuint16 PMDMBGetDACOutputEnable(PMDAxisHandle* axis_handle, PMDuint16* state)
{
    PMDuint16 rc;
    PMDuint16 value;

    rc = PMDReadIO(axis_handle, MB_AMP_ENABLE, &value);
    value >>= 7;
    value &= 1;
    *state = value;

    return rc;
}

/*****************************************************************************
  function: PMDMBReadCardID
  return value: 16 bits split into 4 nibbles as follows

    Bus System, Bits [15..12]
    0 = ISA Bus
    1 = PCI Bus
    2 = CompactPCI
    3 = PC-104
    4 = MIPS
    5 = RS232
    6 = CAN

    Prodigy Board ID, Bit [11]
    1 = Prodigy

    Board Revision Level, Bits [10..8]
    0 = original layout
    1 = revision 1, etc.

    PLD Revision

    Minor Rev., Bits [7..4]
    0 = original
    1 = minor revision 1, etc

    Major Rev., Bits [3..0]
    0 = original
    1 = major revision 1, etc

example:
  0x1840 = PCI, PLD rev 0.4

******************************************************************************/
PMDuint16 PMDMBReadCardID(PMDAxisHandle* axis_handle, PMDuint16* cardid)
{
    return PMDReadIO(axis_handle, MB_PLD, cardid);
}

//*****************************************************************************
    // 0x5562 is the value to write to the watchdog to reset it
    // the watchdog times out after 104 ms.
//*****************************************************************************
PMDuint16 PMDMBSetWatchdog(PMDAxisHandle* axis_handle)
{
    return PMDWriteIO(axis_handle, MB_WATCHDOG, 0x5562);
}

//*****************************************************************************
PMDuint16 PMDMBGetResetCause(PMDAxisHandle* axis_handle, PMDuint16* resetcause)
{
    PMDuint16 rc;
    PMDuint16 value = 0;
    
    rc =  PMDReadIO(axis_handle, MB_RESET_CAUSE, &value);
    value >>= 12;
    *resetcause = value;

    return rc;
}
//*****************************************************************************
PMDuint16 PMDMBClearResetCause(PMDAxisHandle* axis_handle)
{
    PMDuint16 rc;
    PMDuint16 value = 0;
    
    rc =  PMDWriteIO(axis_handle, MB_RESET_CAUSE, value);

    return rc;
}


// PMDSSI (absolute encoder) specific functions

//*****************************************************************************
PMDuint16 PMDMBSetSSIRegister(PMDAxisHandle* axis_handle, PMDSSIResolution resolution, PMDSSIFrequency frequency)
{
    PMDAxis axis = axis_handle->axis;
    PMDuint16 address = MB_SSI_CLOCK + (axis * 2);
    PMDuint16 result;
    PMDuint16 value = 0;

    value = resolution;
    value <<= 2;
    value |= frequency;
    value <<= 10;
    result = PMDWriteIO(axis_handle, address, value);

    return result;
}

//*****************************************************************************
PMDuint16 PMDMBGetSSIRegister(PMDAxisHandle* axis_handle, PMDSSIResolution* resolution, PMDSSIFrequency* frequency)
{
    PMDAxis axis = axis_handle->axis;
    PMDuint16 address = MB_SSI_CLOCK + (axis * 2);
    PMDuint16 result;
    PMDuint16 value = 0;

    result = PMDReadIO(axis_handle, address, &value);
    value >>= 10;
    *frequency = (PMDSSIFrequency)(value & 0x0003);
    value >>= 2;
    *resolution = (PMDSSIResolution)(value & 0x0003);

    return result;
}

//*****************************************************************************
PMDuint16 PMDMBGetSSIAbsolutePosition(PMDAxisHandle* axis_handle, PMDint32* position)
{
    PMDAxis axis = axis_handle->axis;
    PMDuint16 positionhi;
    PMDuint16 positionlo;
    PMDint32 position32;
    PMDuint16 result;
    PMDuint16 address = MB_SSI_POSITION + (axis * 2);

    result = PMDReadIO(axis_handle, address, &positionlo);
    address++;
    result = PMDReadIO(axis_handle, address, &positionhi);

    position32 = positionhi & 0x02FF;
    position32 <<= 16;
    position32 |= positionlo;
    *position = position32;

    return result;
}

//*****************************************************************************
PMDuint16 PMDMBSetActualPositionToSSIPosition(PMDAxisHandle* axis_handle)
{
    PMDint32 position = 0;
    PMDuint16 result;

    result = PMDMBGetSSIAbsolutePosition(axis_handle, &position);
    result = PMDSetActualPosition(axis_handle, position);
    return result;
}

//*****************************************************************************
PMDuint16 PMDMBGetSSIVersion(PMDAxisHandle* axis_handle, PMDuint16* version)
{
    return PMDReadIO(axis_handle, MB_SSI_VERSION, version);
}

//*****************************************************************************
PMDuint16 PMDMBResetSSI(PMDAxisHandle* axis_handle)
{
    return PMDWriteIO(axis_handle, MB_SSI_RESET, 0);
}



