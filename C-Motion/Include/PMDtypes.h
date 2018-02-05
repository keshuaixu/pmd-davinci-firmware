#ifndef PMD_CommonTypes
#define PMD_CommonTypes

//  PMDtypes.h -- common type declarations
//
//  Performance Motion Devices, Inc.
//

#include "pmdecode.h"


// Global definitions

// Macros to control the export of DLL entry points.
// The stdcall calling convention is used so that Visual Basic
// can call these procedures.

#if defined(_WIN32) && (defined(PMD_EXPORTS) || defined(PMD_IMPORTS))

# define PMD_CCONV __stdcall
# ifdef PMD_EXPORTS
// For the library build
#  define PMD_API __declspec(dllexport)
# else
// For library users
#  define PMD_API __declspec(dllimport)
# endif

#else

// For non-Windows builds
# define PMD_CCONV /**/
# define PMD_API extern
#endif

// This macro can be used as the return type for almost all user-callable library procedures.
#define PMDCFunc PMD_API PMDresult PMD_CCONV

#define PMD_MAX_AXES 4

#define PMDnull 0

#define PMDparam                PMDuint32


// common types
typedef long PMDint32;
typedef unsigned long  PMDuint32;
typedef long PMDlong32;
typedef unsigned short PMDuint16;
typedef short PMDint16;
typedef unsigned char  PMDuint8;
typedef PMDErrorCode PMDresult;

#ifndef DWORD
#define DWORD PMDuint32
#endif
#ifndef WORD
#define WORD PMDuint16
#endif
#ifndef BYTE
#define BYTE PMDuint8
#endif
#ifndef NULL
#define NULL 0
#endif


enum {nibbleMask = 0x0F, byteMask = 0xFF};

enum {PMDAxisAtlasMask = 0x20};

enum {
        PMDAxis1 = 0,
        PMDAxis2 = 1,
        PMDAxis3 = 2,
        PMDAxis4 = 3,
        PMDAxisAtlas1 = PMDAxis1 + PMDAxisAtlasMask,
        PMDAxisAtlas2 = PMDAxis2 + PMDAxisAtlasMask,
        PMDAxisAtlas3 = PMDAxis3 + PMDAxisAtlasMask,
        PMDAxisAtlas4 = PMDAxis4 + PMDAxisAtlasMask
};

typedef PMDuint16 PMDAxis;


typedef enum {
        PMDNoAxisMask                                           = 0x00,
        PMDAxis1Mask                                            = 0x01,
        PMDAxis2Mask                                            = 0x02,
        PMDAxis3Mask                                            = 0x04,
        PMDAxis4Mask                                            = 0x08,
} PMDAxisMask;

enum {
        PMDDisable                                              = 0,
        PMDEnable                                               = 1
}; // generic enable switch

typedef enum {
        PMDMotorTypeBrushlessDC3Phase                           = 0,
        PMDMotorTypeBrushlessDC2Phase                           = 1,
        PMDMotorTypeMicrostep3Phase                             = 2,
        PMDMotorTypeMicrostep2Phase                             = 3,
        PMDMotorTypeStep                                        = 4,
        PMDMotorTypeDCBrush                                     = 7
} PMDMotorType;

// Profile Generation
typedef enum {
        PMDTrapezoidalProfile                                   = 0,
        PMDVelocityContouringProfile                            = 1,
        PMDSCurveProfile                                        = 2,
        PMDElectronicGearProfile                                = 3
} PMDProfileMode;

typedef enum {
        PMDNoStopMode                                           = 0,
        PMDAbruptStopMode                                       = 1,
        PMDSmoothStopMode                                       = 2
} PMDStopMode;

// Servo Filter
typedef enum {
        PMDMotionCompleteCommandedPosition                      = 0,
        PMDMotionCompleteActualPosition                         = 1
} PMDMotionCompleteMode;

// Parameter Update & Breakpoints
typedef enum {
        PMDBreakpoint1                                          = 0,
        PMDBreakpoint2                                          = 1
} PMDBreakpoint;

typedef enum {
        PMDBreakpointDisable                                    = 0,
        PMDBreakpointGreaterOrEqualCommandedPosition            = 1,
        PMDBreakpointLessOrEqualCommandedPosition               = 2,
        PMDBreakpointGreaterOrEqualActualPosition               = 3,
        PMDBreakpointLessOrEqualActualPosition                  = 4,
        PMDBreakpointCommandedPositionCrossed                   = 5,
        PMDBreakpointActualPositionCrossed                      = 6,
        PMDBreakpointTime                                       = 7,
        PMDBreakpointEventStatus                                = 8,
        PMDBreakpointActivityStatus                             = 9,
        PMDBreakpointSignalStatus                               = 10,
        PMDBreakpointDriveStatus                                = 11
} PMDBreakpointTrigger;

typedef enum {
        PMDBreakpointNoAction                                   = 0,
        PMDBreakpointActionUpdate                               = 1,
        PMDBreakpointActionAbruptStop                           = 2,
        PMDBreakpointActionSmoothStop                           = 3,
        PMDBreakpointActionMotorOff                             = 4,
        PMDBreakpointActionDisablePositionLoopAndHigherModules  = 5,
        PMDBreakpointActionDisableCurrentLoopAndHigherModules   = 6,
        PMDBreakpointActionDisableMotorOutputAndHigherModules   = 7,
        PMDBreakpointActionAbruptStopWithPositionErrorClear     = 8
} PMDBreakpointAction;

// Status Register Control
typedef enum {
        PMDActivityPhasingInitializedMask                       = 0x0001,
        PMDActivityAtMaximumVelocityMask                        = 0x0002,
        PMDActivityTrackingMask                                 = 0x0004,
        PMDActivityProfileModeMask                              = 0x0038,
        PMDActivityAxisSettledMask                              = 0x0080,
        PMDActivityMotorOnMask                                  = 0x0100,
        PMDActivityPositionCaptureMask                          = 0x0200,
        PMDActivityInMotionMask                                 = 0x0400,
        PMDActivityInPositiveLimitMask                          = 0x0800,
        PMDActivityInNegativeLimitMask                          = 0x1000,
        PMDActivityProfileSegmentMask                           = 0xE000
} PMDActivityMask;

typedef enum {
        PMDEventMotionCompleteMask                              = 0x0001,
        PMDEventWrapAroundMask                                  = 0x0002,
        PMDEventBreakpoint1Mask                                 = 0x0004,
        PMDEventCaptureReceivedMask                             = 0x0008,
        PMDEventMotionErrorMask                                 = 0x0010,
        PMDEventInPositiveLimitMask                             = 0x0020,
        PMDEventInNegativeLimitMask                             = 0x0040,
        PMDEventInstructionErrorMask                            = 0x0080,
        PMDEventDriveDisabledMask                               = 0x0100,
        PMDEventOvertemperatureFaultMask                        = 0x0200,
        PMDEventBusVoltageFaultMask                             = 0x0400,
        PMDEventDriveExceptionMask                              = 0x0400,    //Atlas, MC58113
        PMDEventCommutationErrorMask                            = 0x0800,
        PMDEventCurrentFoldbackMask                             = 0x1000,
        PMDEventBreakpoint2Mask                                 = 0x4000,
        PMDEventAllMask                                         = 0x5FFF
} PMDEventMask;

typedef enum {
        PMDSignalEncoderAMask                                   = 0x0001,
        PMDSignalEncoderBMask                                   = 0x0002,
        PMDSignalEncoderIndexMask                               = 0x0004,
        PMDSignalEncoderHomeMask                                = 0x0008,
        PMDSignalPositiveLimitMask                              = 0x0010,
        PMDSignalNegativeLimitMask                              = 0x0020,
        PMDSignalAxisInMask                                     = 0x0040,
        PMDSignalHallAMask                                      = 0x0080,
        PMDSignalHallBMask                                      = 0x0100,
        PMDSignalHallCMask                                      = 0x0200,
        PMDSignalAxisOutMask                                    = 0x0400,
        PMDSignalStepOutputInvertMask                           = 0x0800,
        PMDSignalMotorDirectionMask                             = 0x1000,
        PMDSignalEnableInMask                                   = 0x2000,
        PMDSignalFaultOutMask                                   = 0x4000
} PMDSignalMask;

// Encoder
typedef enum {
        PMDCaptureSourceIndex                                   = 0,
        PMDCaptureSourceHome                                    = 1,
        PMDCaptureSourceHSI                                     = 2
} PMDCaptureSource;

typedef enum {
        PMDEncoderSourceIncremental                             = 0,
        PMDEncoderSourceParallel                                = 1,
        PMDEncoderSourceNone                                    = 2,
        PMDEncoderSourceLoopback                                = 3,
        PMDEncoderSourcePulseAndDirection                       = 4,
        PMDEncoderSourceHall                                    = 5, // MC58113
        PMDEncoderSourceParallel32                              = 6,
        PMDEncoderSourceAnalogHall                              = 7  // MC58113
} PMDEncoderSource;

// Motor
typedef enum {
        PMDMotorOutputBipolarDAC                                = 0,
        PMDMotorOutputPWMSignMagnitude                          = 1,
        PMDMotorOutputPWM5050Magnitude                          = 2,
        PMDMotorOutputSPIDACOffsetBinary                        = 3,
        PMDMotorOutputUnipolarDAC                               = 4,
        PMDMotorOutputSPIDAC2sComplement                        = 5,
        PMDMotorOutputAtlas                                     = 6,
        PMDMotorOutputPWMHighLow                                = 7, // MC58113,
        PMDMotorOutputPulseAndDirection                         = 8,
        PMDMotorOutputAtlasRecovery                             = 9,
        PMDMotorOutputNone                                      = 10 // MC58113
} PMDMotorOutputMode;

typedef enum {
        PMDMotorOff                                             = 0,
        PMDMotorOn                                              = 1
} PMDMotorMode;

// Commutation
typedef enum {
        PMDCommutationModeSinusoidal                            = 0,
        PMDCommutationModeHallBased                             = 1
} PMDCommutationMode;

typedef enum {
        PMDPhaseInitializeModeAlgorithmic                       = 0,
        PMDPhaseInitializeModeHallBased                         = 1
} PMDPhaseInitializeMode;

typedef enum {
        PMDPhaseCorrectionModeDisabled                          = 0,
        PMDPhaseCorrectionModeIndex                             = 1,
        PMDPhaseCorrectionModeHall                              = 2 // MC58113
} PMDPhaseCorrectionMode;

typedef enum {
        PMDPhasePrescaleOff                                     = 0,
        PMDPhasePrescale64                                      = 1,
        PMDPhasePrescale128                                     = 2,
        PMDPhasePrescale256                                     = 3
} PMDPhasePrescaleMode;

typedef enum {
        PMDPhaseA                                               = 0,
        PMDPhaseB                                               = 1,
        PMDPhaseC                                               = 2
} PMDPhase;

// Trace Operations
typedef enum {
        PMDTraceVariable1                                       = 0,
        PMDTraceVariable2                                       = 1,
        PMDTraceVariable3                                       = 2,
        PMDTraceVariable4                                       = 3
} PMDTraceVariableNumber;

typedef enum {
        PMDTraceModeOneTime                                     = 0,
        PMDTraceModeRollingBuffer                               = 1,
        PMDTraceModeExternalTrigger                             = 0x0100
} PMDTraceMode;

typedef enum {
        PMDTraceNone                                            = 0,
        PMDTracePositionError                                   = 1,
        PMDTraceCommandedPosition                               = 2,
        PMDTraceCommandedVelocity                               = 3,
        PMDTraceCommandedAcceleration                           = 4,
        PMDTraceActualPosition                                  = 5,
        PMDTraceActualVelocity                                  = 6,
        PMDTraceActiveMotorCommand                              = 7,
        PMDTraceMotionProcessorTime                             = 8,
        PMDTraceCaptureRegister                                 = 9,
        PMDTraceActualVelocity32                                = 83,
        PMDTraceRawEncoderValue                                 = 84,

        PMDTracePositionLoopIntegralSum                         = 10,
        PMDTracePositionLoopIntegralContribution                = 57,
        PMDTracePositionLoopDerivative                          = 11,
        PMDTracePIDOutput                                       = 64,
        PMDTraceBiquad1Output                                   = 65,

        PMDTraceEventStatusRegister                             = 12,
        PMDTraceActivityStatusRegister                          = 13,
        PMDTraceSignalStatusRegister                            = 14,
        PMDTraceDriveStatusRegister                             = 56,
        PMDTraceDriveFaultStatusRegister                        = 79,
        PMDTraceAtlasSPIStatus                                  = 80,

        PMDTracePhaseAngle                                      = 15,
        PMDTracePhaseOffset                                     = 16,
        PMDTracePhaseACommand                                   = 17,
        PMDTracePhaseBCommand                                   = 18,
        PMDTracePhaseCCommand                                   = 19,

        PMDTraceAnalogInput0                                    = 20,
        PMDTraceAnalogInput1                                    = 21,
        PMDTraceAnalogInput2                                    = 22,
        PMDTraceAnalogInput3                                    = 23,
        PMDTraceAnalogInput4                                    = 24,
        PMDTraceAnalogInput5                                    = 25,
        PMDTraceAnalogInput6                                    = 26,
        PMDTraceAnalogInput7                                    = 27,
        PMDTracePhaseAngleScaled                                = 29,

        PMDTraceCurrentLoopAReference                           = 66,// 42h
        PMDTraceCurrentLoopAError                               = 30,
        PMDTraceCurrentLoopActualCurrentA                       = 31,
        PMDTraceCurrentLoopAIntegratorSum                       = 32,
        PMDTraceCurrentLoopAIntegralContribution                = 33,
        PMDTraceCurrentLoopAOutput                              = 34,

        PMDTraceCurrentLoopBReference                           = 67,// 43h
        PMDTraceCurrentLoopBError                               = 35,
        PMDTraceCurrentLoopActualCurrentB                       = 36,
        PMDTraceCurrentLoopBIntegratorSum                       = 37,
        PMDTraceCurrentLoopBIntegralContribution                = 38,
        PMDTraceCurrentLoopBOutput                              = 39,

        PMDTraceFOCDReference                                   = 40,// 28h
        PMDTraceFOCDError                                       = 41,
        PMDTraceFOCDFeedback                                    = 42,
        PMDTraceFOCDIntegratorSum                               = 43,
        PMDTraceFOCDIntegralContribution                        = 44,
        PMDTraceFOCDOutput                                      = 45,

        PMDTraceFOCQReference                                   = 46,// 2Eh
        PMDTraceFOCQError                                       = 47,
        PMDTraceFOCQFeedback                                    = 48,
        PMDTraceFOCQIntegratorSum                               = 49,
        PMDTraceFOCQIntegralContribution                        = 50,
        PMDTraceFOCQOutput                                      = 51,

        PMDTraceFOCAlphaOutput                                  = 52,// 34h
        PMDTraceFOCBetaOutput                                   = 53,
        PMDTraceFOCActualCurrentA                               = 31,
        PMDTraceFOCActualCurrentB                               = 36,

        PMDTraceBusVoltage                                      = 54,// 36h
        PMDTraceTemperature                                     = 55,
        PMDTraceI2tEnergy                                       = 68,
        // Atlas
        PMDTraceLegCurrentA                                     = 69,
        PMDTraceLegCurrentB                                     = 70,
        PMDTraceLegCurrentC                                     = 71,
        PMDTraceLegCurrentD                                     = 72,
        PMDTracePhaseCurrentAlpha                               = 73,
        PMDTracePhaseCurrentBeta                                = 74,
        PMDTracePWMOutputA                                      = 75,
        PMDTracePWMOutputB                                      = 76,
        PMDTracePWMOutputC                                      = 77,

} PMDTraceVariable;

typedef enum {
        PMDTraceConditionImmediate                              = 0,
        PMDTraceConditionNextUpdate                             = 1,
        PMDTraceConditionEventStatus                            = 2,
        PMDTraceConditionActivityStatus                         = 3,
        PMDTraceConditionSignalStatus                           = 4,
        PMDTraceConditionDriveStatus                            = 5,
        PMDTraceConditionSPICommand                             = 6 // Atlas
} PMDTraceCondition;

typedef enum {
        PMDTraceTriggerStateLow                                 = 0,
        PMDTraceTriggerStateHigh                                = 1
} PMDTraceTriggerState;

typedef enum {
        PMDTraceStatusMode                                      = 0x0001,
        PMDTraceStatusActivity                                  = 0x0002,
        PMDTraceStatusDataWrap                                  = 0x0004,
        PMDTraceStatusOverrun                                   = 0x0008,  // Atlas
        PMDTraceStatusDataAvailable                             = 0x0010   // Atlas
} PMDTraceStatusMask;

// Miscellaneous
enum {
        PMDActivityPhasingInitializedBit                        = 0,
        PMDActivityAtMaximumVelocityBit                         = 1,
        PMDActivityTrackingBit                                  = 2,
        PMDActivityAxisSettledBit                               = 7,
        PMDActivityMotorModeBit                                 = 8,
        PMDActivityPositionCaptureBit                           = 9,
        PMDActivityInMotionBit                                  = 10,
        PMDActivityInPositiveLimitBit                           = 11,
        PMDActivityInNegitiveLimitBit                           = 12
};

enum {
        PMDEventMotionCompleteBit                               = 0,
        PMDEventWrapAroundBit                                   = 1,
        PMDEventBreakpoint1Bit                                  = 2,
        PMDEventCaptureReceivedBit                              = 3,
        PMDEventMotionErrorBit                                  = 4,
        PMDEventPositiveLimitBit                                = 5,
        PMDEventNegativeLimitBit                                = 6,
        PMDEventInstructionErrorBit                             = 7,
        PMDEventDriveDisabledBit                                = 8,
        PMDEventOvertemperatureFaultBit                         = 9,
        PMDEventBusVoltageFaultBit                              = 10, // ION
        PMDEventDriveExceptionBit                               = 10, // Atlas, MC58113
        PMDEventCommutationErrorBit                             = 11,
        PMDEventCurrentFoldbackBit                              = 12,
        PMDEventBreakpoint2Bit                                  = 14
};

enum {
        PMDSignalEncoderABit                                    = 0,
        PMDSignalEncoderBBit                                    = 1,
        PMDSignalEncoderIndexBit                                = 2,
        PMDSignalEncoderHomeBit                                 = 3,
        PMDSignalPositiveLimitBit                               = 4,
        PMDSignalNegativeLimitBit                               = 5,
        PMDSignalAxisInBit                                      = 6,
        PMDSignalHallABit                                       = 7,
        PMDSignalHallBBit                                       = 8,
        PMDSignalHallCBit                                       = 9,
        PMDSignalAxisOutBit                                     = 10,
        PMDSignalStepOutputBit                                  = 11,
        PMDSignalMotorDirectionBit                              = 12,
        PMDSignalEnableInBit                                    = 13,
        PMDSignalFaultOutBit                                    = 14
};

typedef enum {
        PMDAxisOutRegisterNone                                  = 0,
        PMDAxisOutRegisterEventStatus                           = 1,
        PMDAxisOutRegisterActivityStatus                        = 2,
        PMDAxisOutRegisterSignalStatus                          = 3,
        PMDAxisOutRegisterDriveStatus                           = 4
} PMDAxisOutRegister;

typedef enum {
        PMDAxisOff                                              = 0,
        PMDAxisOn                                               = 1
} PMDAxisMode;

typedef enum {
        PMDSerialBaud1200                                       = 0,
        PMDSerialBaud2400                                       = 1,
        PMDSerialBaud9600                                       = 2,
        PMDSerialBaud19200                                      = 3,
        PMDSerialBaud57600                                      = 4,
        PMDSerialBaud115200                                     = 5,
        PMDSerialBaud230400                                     = 6,
        PMDSerialBaud460800                                     = 7
} PMDSerialBaud;

typedef enum {
        PMDSerialStopBits1                                      = 0,
        PMDSerialStopBits2                                      = 1
} PMDSerialStopBits;

typedef enum {
        PMDSerialProtocolPoint2Point                            = 0,
        PMDSerialProtocolMultiDropUsingIdleLineDetection        = 1,
        PMDSerialProtocolMultiDropUsingAddressBit               = 2,
        PMDSerialProtocolMultiDropUsingIdleLineDetectionOld     = 3
} PMDSerialProtocol;

typedef enum {
        PMDSerialParityNone                                     = 0,
        PMDSerialParityOdd                                      = 1,
        PMDSerialParityEven                                     = 2
} PMDSerialParity;

typedef enum {
        PMDCANBaud1000000                                       = 0,
        PMDCANBaud800000                                        = 1,
        PMDCANBaud500000                                        = 2,
        PMDCANBaud250000                                        = 3,
        PMDCANBaud125000                                        = 4,
        PMDCANBaud50000                                         = 5,
        PMDCANBaud20000                                         = 6,
        PMDCANBaud10000                                         = 7
} PMDCANBaud;

typedef enum {
        PMDSPIModeRisingEdge                                    = 0,
        PMDSPIModeRisingEdgeDelay                               = 1,
        PMDSPIModeFallingEdge                                   = 2,
        PMDSPIModeFallingEdgeDelay                              = 3
} PMDSPIMode;

typedef enum {
        PMDMotorTypeVersionBrushedServo                         = 1,
        PMDMotorTypeVersionBrushlessServo                       = 3,
        PMDMotorTypeVersionMicroStepping                        = 4,
        PMDMotorTypeVersionStepping                             = 5,
        PMDMotorTypeVersionAllMotor                             = 8,
        PMDMotorTypeVersionAnyMotor                             = 9
} PMDMotorTypeVersion;

typedef enum {
        PMDFamilyFirstGen                                       = 1,
        PMDFamilyNavigator                                      = 2,
        PMDFamilyPilot                                          = 3,
        PMDFamilyMagellan                                       = 5,
        PMDFamilyMotorControl                                   = 7,
        PMDFamilyMagellanION                                    = 9
} PMDFamily;

//ION and Atlas specific
typedef enum {
        PMDCurrentControlModeCurrentLoop                        = 0,
        PMDCurrentControlModeFOC                                = 1,
        PMDCurrentControlModeThirdLegFloating                   = 2
} PMDCurrentControlMode;

typedef enum {
        PMDFOCDirect                                            = 0,
        PMDFOCQuadrature                                        = 1,
        PMDFOCBoth                                              = 2
} PMDFOC;

typedef enum {
        PMDFOCProportionalGain                                  = 0,
        PMDFOCIntegralGain                                      = 1,
        PMDFOCIntegralSumLimit                                  = 2
} PMDFOCParameter;

typedef enum {
        PMDCurrentLoopPhaseA                                    = 0,
        PMDCurrentLoopPhaseB                                    = 1,
        PMDCurrentLoopBoth                                      = 2
} PMDCurrentLoop;

typedef enum {
        PMDCurrentLoopProportionalGain                          = 0,
        PMDCurrentLoopIntegralGain                              = 1,
        PMDCurrentLoopIntegralSumLimit                          = 2
} PMDCurrentLoopParameter;

typedef enum {
        PMDCurrentLoopValueNodeReference                        = 0,
        PMDCurrentLoopValueNodeActualCurrent                    = 1,
        PMDCurrentLoopValueNodeError                            = 2,
        PMDCurrentLoopValueNodeIntegratorSum                    = 3,
        PMDCurrentLoopValueNodeIntegralContribution             = 5,
        PMDCurrentLoopValueNodeOutput                           = 6,
        PMDCurrentLoopValueNodeI2tEnergy                        = 10
} PMDCurrentLoopValueNode;

typedef enum {
        PMDFOCValueNodeReference                                = 0,
        PMDFOCValueNodeFeedback                                 = 1,
        PMDFOCValueNodeError                                    = 2,
        PMDFOCValueNodeIntegratorSum                            = 3,
        PMDFOCValueNodeIntegralContribution                     = 5,
        PMDFOCValueNodeOutput                                   = 6,
        PMDFOCValueNodeFOCOutput                                = 7,
        PMDFOCValueNodeActualCurrent                            = 8,
        PMDFOCValueNodeI2tEnergy                                = 10
} PMDFOCValueNode;

typedef enum {
        PMDDriveStatusInFoldback                                = 1 << 1,
        PMDDriveStatusOverTemperature                           = 1 << 2,
        PMDDriveStatusShuntActive                               = 1 << 3,
        PMDDriveStatusInHolding                                 = 1 << 4,
        PMDDriveStatusOverVoltage                               = 1 << 5,
        PMDDriveStatusUnderVoltage                              = 1 << 6,
        PMDDriveStatusDisabled                                  = 1 << 7,
        PMDDriveStatusOutputClipped                             = 1 << 12,
        PMDDriveStatusAtlasNotConnected                         = 1 << 15
} PMDDriveStatusMask;

typedef enum {
        PMDDriveFaultStatusShortCircuit                         = 1 << 0,
        PMDDriveFaultStatusGround                               = 1 << 1, // ION only
        PMDDriveFaultStatusExternalLogic                        = 1 << 2, // ION only
        PMDDriveFaultStatusOpmodeMismatch                       = 1 << 3, // Atlas only
        PMDDriveFaultStatusInternalLogic                        = 1 << 4,
        PMDDriveFaultStatusOverVoltage                          = 1 << 5,
        PMDDriveFaultStatusUnderVoltage                         = 1 << 6,
        PMDDriveFaultStatusDisabled                             = 1 << 7,
        PMDDriveFaultStatusFoldback                             = 1 << 8,
        PMDDriveFaultStatusOverTemperature                      = 1 << 9,
        PMDDriveFaultStatusAtlasDetectedSPIChecksum             = 1 << 10, // Atlas only
        PMDDriveFaultStatusWatchdog                             = 1 << 11, // Atlas only
        PMDDriveFaultStatusPWMDisabled                          = 1 << 11, // by PWMOutputDisable signal
        PMDDriveFaultStatusMagellanDetectedSPIChecksum          = 1 << 14, // Atlas only
        PMDDriveFaultStatusMotorTypeMismatch                    = 1 << 15  // Atlas only
} PMDDriveFaultStatusMask;

typedef enum {
        PMDCurrentHoldingMotorLimit                             = 0,
        PMDCurrentHoldingDelay                                  = 1,
        PMDCurrentDriveCurrent                                  = 2,
} PMDCurrent;

// PMDHoldingCurrent has been replaced by PMDCurrent but is left in for compatibility
typedef enum {
        PMDHoldingMotorLimit,
        PMDHoldingDelay
} PMDHoldingCurrent;

typedef enum {
        PMDFoldbackContinuousCurrentLimit,
        PMDFoldbackI2tThreshold
} PMDFoldbackCurrent;

typedef enum {
        PMDOperatingModeAxisEnabledMask                         = 0x0001,
        PMDOperatingModeMotorOutputEnabledMask                  = 0x0002,
        PMDOperatingModeCurrentControlEnabledMask               = 0x0004,
        PMDOperatingModePositionLoopEnabledMask                 = 0x0010,
        PMDOperatingModeTrajectoryEnabledMask                   = 0x0020,
        PMDOperatingModeAllEnabledMask                          = 0x0037,
} PMDOperatingModeMask;

typedef enum {
        PMDEventActionEventImmediate                            = 0,
        PMDEventActionEventPositiveLimit                        = 1,
        PMDEventActionEventNegativeLimit                        = 2,
        PMDEventActionEventMotionError                          = 3,
        PMDEventActionEventCurrentFoldback                      = 4
} PMDEventActionEvent;

typedef enum {
        PMDEventActionNone                                      = 0,
        PMDEventActionAbruptStop                                = 2,
        PMDEventActionSmoothStop                                = 3,
        PMDEventActionDisablePositionLoopAndHigherModules       = 5,
        PMDEventActionDisableCurrentLoopAndHigherModules        = 6,
        PMDEventActionDisableMotorOutputAndHigherModules        = 7,
        PMDEventActionAbruptStopWithPositionErrorClear          = 8
} PMDEventAction;

typedef enum {
        PMDUpdateTrajectoryMask                                 = 0x0001,
        PMDUpdatePositionLoopMask                               = 0x0002,
        PMDUpdateCurrentLoopMask                                = 0x0008
} PMDUpdateMask;

typedef enum {
        PMDPositionLoopValueNodeIntegratorSum                   = 0,
        PMDPositionLoopValueNodeIntegralContribution            = 1,
        PMDPositionLoopValueNodeDerivative                      = 2,
        PMDPositionLoopValueNodeBiquad1Input                    = 3,
        PMDPositionLoopValueNodeBiquad2Input                    = 4,
} PMDPositionLoopValueNode;

typedef enum {
        PMDPositionLoopProportionalGain                         = 0,
        PMDPositionLoopIntegratorGain                           = 1,
        PMDPositionLoopIntegratorLimit                          = 2,
        PMDPositionLoopDerivativeGain                           = 3,
        PMDPositionLoopDerivativeTime                           = 4,
        PMDPositionLoopOutputGain                               = 5,
        PMDPositionLoopVelocityFeedforwardGain                  = 6,
        PMDPositionLoopAccelerationFeedforwardGain              = 7,
        PMDPositionLoopBiquad1EnableFilter                      = 8,
        PMDPositionLoopBiquad1B0                                = 9,
        PMDPositionLoopBiquad1B1                                = 10,
        PMDPositionLoopBiquad1B2                                = 11,
        PMDPositionLoopBiquad1A1                                = 12,
        PMDPositionLoopBiquad1A2                                = 13,
        PMDPositionLoopBiquad1K                                 = 14,
        PMDPositionLoopBiquad2EnableFilter                      = 15,
        PMDPositionLoopBiquad2B0                                = 16,
        PMDPositionLoopBiquad2B1                                = 17,
        PMDPositionLoopBiquad2B2                                = 18,
        PMDPositionLoopBiquad2A1                                = 19,
        PMDPositionLoopBiquad2A2                                = 20,
        PMDPositionLoopBiquad2K                                 = 21
} PMDPositionLoop;

typedef enum {
        PMDOvervoltageLimit,
        PMDUndervoltageLimit
} PMDVoltageLimit;

typedef enum {
        PMDProductTypeUnknown                                   = 0,
        PMDProductTypeProdigy                                   = 1,
        PMDProductTypeProdigyCME                                = 2,
        PMDProductTypeMagellan                                  = 3,
        PMDProductTypeION                                       = 4,
        PMDProductTypeIONCME                                    = 5
} PMDProductType;                                               

typedef enum {                                                  
        PMDIONTypeA                                             = 1,
        PMDIONTypeB                                             = 0,
        PMDIONTypeD                                             = 2,
        PMDIONTypeCME                                           = 3
} PMDIONType;

// Atlas & MC58113 specific types
typedef enum {
        PMDDriveNVRAMModeAtlas                                  = 0,
        PMDDriveNVRAMMode58113                                  = 0x100, 
        PMDDriveNVRAMErase                                      = 1,
        PMDDriveNVRAMWrite                                      = 2,
        PMDDriveNVRAMBlockWriteBegin                            = 3,
        PMDDriveNVRAMBlockWriteEnd                              = 4,
        PMDDriveNVRAMSkip                                       = 8
} PMDDriveNVRAMOption;

typedef enum {
        PMDDriveFaultParameterOverVoltageLimit                  = 0,
        PMDDriveFaultParameterUnderVoltageLimit                 = 1,
        PMDDriveFaultParameterRecoveryMode                      = 2,
        PMDDriveFaultParameterWatchdogLimit                     = 3,
        PMDDriveFaultParameterTemperatureLimit                  = 4,
        PMDDriveFaultParameterTemperatureHysteresis             = 5,
        PMDDriveFaultParameterShuntVoltageLimit                 = 8,  // MC58113
        PMDDriveFaultParameterShuntDuty                         = 9,  // MC58113
        PMDDriveFaultParameterBusCurrentSupplyLimit             = 10, // MC58113
        PMDDriveFaultParameterBusCurrentReturnLimit             = 11, // MC58113
} PMDDriveFaultParameter;

typedef enum {
        PMDDrivePWMLimit                                        = 0,
        PMDDrivePWMDeadTime                                     = 1,
        PMDDrivePWMSignalSense                                  = 2,
        PMDDrivePWMFrequency                                    = 3,
        PMDDrivePWMRefreshPeriod                                = 4,
        PMDDrivePWMRefreshTime                                  = 5,
        PMDDrivePWMCurrentSenseTime                             = 6
} PMDDrivePWM;

typedef enum {
        PMDAnalogCalibrationCurrentLegA                         = 0,
        PMDAnalogCalibrationCurrentLegB                         = 1,
        PMDAnalogCalibrationCurrentLegC                         = 2,
        PMDAnalogCalibrationCurrentLegD                         = 3
} PMDAnalogCalibration;

typedef enum {
        PMDFeedbackParameterModulus                             = 0
} PMDFeedbackParameter;

typedef enum {
        PMDDriveValueBusVoltage                                 = 0,
        PMDDriveValueTemperature                                = 1,
        PMDDriveValueBusCurrentSupply                           = 2,
        PMDDriveValueBusCurrentReturn                           = 3
} PMDDriveValueMask;


#endif
