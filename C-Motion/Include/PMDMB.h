#ifndef PMD_MotionBoard
#define PMD_MotionBoard

//  pmdmb.h -- MotionBoard specific API
//
//  Performance Motion Devices, Inc.
//

#include "c-motion.h"

#if defined(__cplusplus)
extern "C" {
#endif


PMDuint16 PMDMBWriteDigitalOutput(PMDAxisHandle* axis_handle, PMDuint16 state);
PMDuint16 PMDMBReadDigitalOutput(PMDAxisHandle* axis_handle, PMDuint16* state);
PMDuint16 PMDMBReadDigitalInput(PMDAxisHandle* axis_handle, PMDuint16* state);
PMDuint16 PMDMBSetAmplifierEnable(PMDAxisHandle* axis_handle, PMDuint16 axismask, PMDuint16 state);
PMDuint16 PMDMBGetAmplifierEnable(PMDAxisHandle* axis_handle, PMDuint16* state);
PMDuint16 PMDMBSetDACOutputEnable(PMDAxisHandle* axis_handle, PMDuint16 state);
PMDuint16 PMDMBGetDACOutputEnable(PMDAxisHandle* axis_handle, PMDuint16* state);
PMDuint16 PMDMBReadCardID(PMDAxisHandle* axis_handle, PMDuint16* cardid);
PMDuint16 PMDMBSetWatchdog(PMDAxisHandle* axis_handle);
PMDuint16 PMDMBGetResetCause(PMDAxisHandle* axis_handle, PMDuint16* resetcause);
PMDuint16 PMDMBClearResetCause(PMDAxisHandle* axis_handle);

typedef enum
{
    PMDResetCauseException    = 1 << 10,
    PMDResetCauseSysWatchdog  = 1 << 11,
    PMDResetCauseHardReset    = 1 << 12,
    PMDResetCauseUnderVoltage = 1 << 13,
    PMDResetCauseExternal     = 1 << 14,
    PMDResetCauseWatchdog     = 1 << 15,
} PMDResetCause;

// PMDSSI specific

typedef enum
{
    PMDSSIFreq1100000 = 0,
    PMDSSIFreq550000,
    PMDSSIFreq275000,
    PMDSSIFreq137500
} PMDSSIFrequency;

typedef enum
{
    PMDSSIRes10 = 0,
    PMDSSIRes12,
    PMDSSIRes13,
    PMDSSIRes25
} PMDSSIResolution;


PMDuint16 PMDMBSetSSIRegister(PMDAxisHandle* axis_handle, PMDSSIResolution resolution, PMDSSIFrequency frequency);
PMDuint16 PMDMBGetSSIRegister(PMDAxisHandle* axis_handle, PMDSSIResolution* resolution, PMDSSIFrequency* frequency);
PMDuint16 PMDMBGetSSIAbsolutePosition(PMDAxisHandle* axis_handle, PMDint32* position);
PMDuint16 PMDMBSetActualPositionToSSIPosition(PMDAxisHandle* axis_handle);
PMDuint16 PMDMBGetSSIVersion(PMDAxisHandle* axis_handle, PMDuint16* version);
PMDuint16 PMDMBResetSSI(PMDAxisHandle* axis_handle);

#if defined(__cplusplus)
}
#endif

#endif
