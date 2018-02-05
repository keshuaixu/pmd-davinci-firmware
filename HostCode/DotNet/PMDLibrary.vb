Imports System.Runtime.InteropServices


Public Module PMD

    '*** C library constant declarations ***

    Public Const PMD_WAITFOREVER = &HFFFFFFFF&
    Public Const DEFAULT_ETHERNET_PORT = 40100

    '*** C library enum declarations ***

    Public Enum PMDDeviceType
        None = 0
        ResourceProtocol = 1
        MotionProcessor = 2
    End Enum

    Public Enum PMDSerialPort
        COM1 = 0
        COM2 = 1
        COM3 = 2
    End Enum

    Public Enum PMDInterfaceType
        None = 0
        Parallel = 1
        PCI = 2
        ISA = 3
        Serial = 4
        CAN = 5
        TCP = 6
        UDP = 7
        USB = 8
    End Enum

    ' NB this enum is not present in the C include files
    Public Enum PMDInstructionError
        NoError = 0
        ProcessorReset = 1
        InvalidInstruction = 2
        InvalidAxis = 3
        InvalidParameter = 4
        TraceRunning = 5
        BlockOutOfBounds = 7
        TraceBufferZero = 8
        BadSerialChecksum = 9
        InvalidNegativeValue = 11
        InvalidParameterChange = 12
        InvalidMoveAfterEventTriggeredStop = 13
        InvalidMoveIntoLimit = 14
        InvalidOperatingModeRestoreAfterEventTriggeredChange = 16
        InvalidOperatingModeForCommand = 17
        BadState = 18
        HardFault = 19
        AtlasNotDetected = 20
        BadSPIChecksum = 21
        InvalidSPIprotocol = 22
        InvalidTorqueCommand = 24
        BadFlashChecksum = 25
        InvalidFlashModeCommand = 26
        ReadOnlyBuffer = 27
        InitializationOnlyCommand = 28
    End Enum

    Public Enum PMDDataSize
        Size8Bit = 1
        Size16Bit = 2
        Size32Bit = 4
    End Enum

    Public Enum PMDMemoryType
        DPRAM = 0
        NVRAM = 1
    End Enum

    Public Enum PMDTaskState
        NoCode = 1
        NotStarted = 2
        Running = 3
        Aborted = 4
    End Enum

    Public Enum PMDDefault
        CP_Serial = &H101&
        CP_MotorType = &H102&
        IPAddress = &H303&
        NetMask = &H304&
        Gateway = &H305&
        IPPort = &H106&
        MACH = &H307&
        MACL = &H308&
        COM1 = &H10E&
        COM2 = &H10F&
        RS485Duplex = &H110&
        CAN = &H111&
        AutoStartMode = &H114&
        DebugSource = &H117&
        ConsoleIntfType = &H118&
        ConsoleIntfAddr = &H319&
        ConsoleIntfPort = &H11A&
        Factory = &HFFFF&
    End Enum

    Public Enum PMDVoltageLimit
        OvervoltageLimit = 0
        UndervoltageLimit = 1
    End Enum

    Public Enum PMDPositionLoop
        ProportionalGain = 0
        IntegratorGain = 1
        IntegratorLimit = 2
        DerivativeGain = 3
        DerivativeTime = 4
        OutputGain = 5
        VelocityFeedforwardGain = 6
        AccelerationFeedforwardGain = 7
        Biquad1EnableFilter = 8
        Biquad1B0 = 9
        Biquad1B1 = 10
        Biquad1B2 = 11
        Biquad1A1 = 12
        Biquad1A2 = 13
        Biquad1K = 14
        Biquad2EnableFilter = 15
        Biquad2B0 = 16
        Biquad2B1 = 17
        Biquad2B2 = 18
        Biquad2A1 = 19
        Biquad2A2 = 20
        Biquad2K = 21
    End Enum

    Public Enum PMDPositionLoopValueNode
        IntegratorSum = 0
        IntegralContribution = 1
        Derivative = 2
        Biquad1Input = 3
        Biquad2Input = 4
    End Enum

    Public Enum PMDUpdateMask
        TrajectoryMask = &H1&
        PositionLoopMask = &H2&
        CurrentLoopMask = &H8&
    End Enum

    Public Enum PMDDriveFaultStatusMask
        ShortCircuitFaultMask = &H1&
        GroundFaultMask = &H2&
        ExternalLogicFaultMask = &H4&
        OpModeMismatchMask = &H8&
        OvervoltageFaultMask = &H20&
        UndervoltageFaultMask = &H40&
        DisabledMask = &H80&
        FoldbackMask = &H100&
        OverTemperatureMask = &H200&
        AtlasDetectedSPIChecksumMask = &H400&
        WatchdogMask = &H800&
        PWMDisabled = &H1000&
        MagellanDetectedSPIChecksumMask = &H4000&
        MotorTypeMismatchMask = &H8000&
    End Enum

    Public Enum PMDDriveFaultParameter
        OverVoltageLimit = 0
        UnderVoltageLimit = 1
        RecoveryMode = 2
        WatchdogLimit = 3
        TemperatureLimit = 4
        TemperatureHysteresis = 5
        ShuntVoltageLimit = 8
        ShuntDuty = 9
        BusCurrentSupplyLimit = 10
        BusCurrentReturnLimit = 11
    End Enum

    Public Enum PMDNVRAM
        NVRAMMode = 0
        NVRAMMode58113 = &H100&
        EraseNVRAM = 1
        Write = 2
        BlockWriteBegin = 3
        BlockWriteEnd = 4
        Skip = 8
    End Enum

    Public Enum PMDDrivePWM
        Limit = 0
        DeadTime = 1
        SignalSense = 2
        Frequency = 3
        RefreshPeriod = 4
        RefreshTime = 5
        CurrentSenseTime = 6
    End Enum

    Public Enum PMDDriveValue
        BusVoltage = 0
        Temperature = 1
        BusCurrentSupply = 2
        BusCurrentReturn = 3
    End Enum

    Public Enum PMDAnalogCalibration
        CurrentLegA = 0
        CurrentLegB = 1
        CurrentLegC = 2
        CurrentLegD = 3
    End Enum

    Public Enum PMDEventAction
        None = 0
        AbruptStop = 2
        SmoothStop = 3
        DisablePositionLoopAndHigherModules = 5
        DisableCurrentLoopAndHigherModules = 6
        DisableMotorOutputAndHigherModules = 7
        AbruptStopWithPositionErrorClear = 8
    End Enum

    Public Enum PMDEventActionEvent
        Immediate = 0
        PositiveLimit = 1
        NegativeLimit = 2
        MotionError = 3
        CurrentFoldback = 4
    End Enum

    Public Enum PMDOperatingModeMask
        AxisEnabledMask = &H1&
        MotorOutputEnabledMask = &H2&
        CurrentControlEnabledMask = &H4&
        PositionLoopEnabledMask = &H10&
        TrajectoryEnabledMask = &H20&
        AllEnabledMask = &H37&
    End Enum

    Public Enum PMDFoldbackCurrent
        ContinuousCurrentLimit = 0
    End Enum

    Public Enum PMDHoldingCurrent
        MotorLimit = 0
        Delay = 1
        DriveCurrent = 2
    End Enum

    Public Enum PMDCurrent
        HoldingCurrent = 0
        Delay = 1
        DriveCurrent = 2
    End Enum

    Public Enum PMDDriveStatusMask
        InFoldbackMask = &H2&
        Overtemperature = &H4&
        ShuntActive = &H8&
        InHolding = &H10&
        Overvoltage = &H20&
        Undervoltage = &H40&
        Disabled = &H80&
        OutputClipped = &H1000&
        AtlasNotConnected = &H8000&
    End Enum

    Public Enum PMDFOCValueNode
        Reference = 0
        Feedback = 1
        FOCError = 2
        IntegratorSum = 3
        IntegralContribution = 5
        Output = 6
        FOCOutput = 7
        ActualCurrent = 8
    End Enum

    Public Enum PMDCurrentLoopParameter
        ProportionalGain = 0
        IntegralGain = 1
        IntegralSumLimit = 2
    End Enum

    Public Enum PMDCurrentLoopNumber
        PhaseA = 0
        PhaseB = 1
        Both = 2
    End Enum

    Public Enum PMDCurrentLoopValueNode
        Reference = 0
        ActualCurrent = 1
        CurrentError = 2
        IntegratorSum = 3
        IntegralContribution = 5
        Output = 6
    End Enum

    Public Enum PMDFOCLoopParameter
        ProportionalGain = 0
        IntegralGain = 1
        IntegralSumLimit = 2
    End Enum

    Public Enum PMDFOC_LoopNumber
        Direct = 0
        Quadrature = 1
        Both = 2
    End Enum

    Public Enum PMDCurrentControlMode
        CurrentLoop = 0
        FOC = 1
    End Enum

    Public Enum PMDMotorTypeVersion
        BrushedServo = 1
        BrushlessServo = 3
        MicroStepping = 4
        Stepping = 5
        AllMotor = 8
        IONMotor = 9
    End Enum

    Public Enum PMDCANBaud
        Baud1000000 = 0
        Baud800000 = 1
        Baud500000 = 2
        Baud250000 = 3
        Baud125000 = 4
        Baud50000 = 5
        Baud20000 = 6
        Baud10000 = 7
    End Enum

    Public Enum PMDSerialParity
        None = 0
        Odd = 1
        Even = 2
    End Enum

    Public Enum PMDSerialProtocol
        Point2Point = 0
        MultiDropUsingIdleLineDetection = 1
        MultiDropUsingAddressBit = 2
        MultiDropUsingIdleLineDetectionOld = 3
    End Enum

    Public Enum PMDSerialStopBits
        Bits1 = 0
        Bits2 = 1
    End Enum

    Public Enum PMDSerialBaud
        Baud1200 = 0
        Baud2400 = 1
        Baud9600 = 2
        Baud19200 = 3
        Baud57600 = 4
        Baud115200 = 5
        Baud230400 = 6
        Baud460800 = 7
    End Enum

    Public Enum PMDSPIMode
        RisingEdge = 0
        RisingEdgeDelay = 1
        FallingEdge = 2
        FallingEdgeDelay = 3
    End Enum

    Public Enum PMDSynchronizationMode
        Disabled = 0
        Master = 1
        Slave = 2
    End Enum

    Public Enum PMDAxisMode
        ModeOff = 0
        ModeOn = 1
    End Enum

    Public Enum PMDAxisOutRegister
        None = 0
        EventStatus = 1
        ActivityStatus = 2
        SignalStatus = 3
        DriveStatus = 4
    End Enum

    Public Enum PMDTraceStatusMask
        Mode = &H1&
        Activity = &H2&
        DataWrap = &H4&
    End Enum

    Public Enum PMDTraceTriggerState
        Low = 0
        High = 1
    End Enum

    Public Enum PMDTraceCondition
        Immediate = 0
        NextUpdate = 1
        EventStatus = 2
        ActivityStatus = 3
        SignalStatus = 4
        DriveStatus = 5
    End Enum

    Public Enum PMDTraceVariable
        None = 0
        PositionError = 1
        CommandedPosition = 2
        CommandedVelocity = 3
        CommandedAcceleration = 4
        ActualPosition = 5
        ActualVelocity = 6
        ActualVelocity32 = 83   'MC58113
        RawEncoderValue = 84    'MC58113
        ActiveMotorCommand = 7
        MotionProcessorTime = 8
        CaptureRegister = 9
        PositionLoopIntegralSum = 10
        PositionLoopIntegralContribution = 57
        PositionLoopDerivative = 11
        PIDOutput = 64
        Biquad1Output = 65
        EventStatusRegister = 12
        ActivityStatusRegister = 13
        SignalStatusRegister = 14
        DriveStatusRegister = 56
        DriveFaultStatusRegister = 79
        AtlasSPIStatus = 80
        PhaseAngle = 15
        PhaseOffset = 16
        PhaseACommand = 17
        PhaseBCommand = 18
        PhaseCCommand = 19
        AnalogInput0 = 20
        AnalogInput1 = 21
        AnalogInput2 = 22
        AnalogInput3 = 23
        AnalogInput4 = 24
        AnalogInput5 = 25
        AnalogInput6 = 26
        AnalogInput7 = 27
        PhaseAngleScaled = 29
        CurrentLoopAReference = 66
        CurrentLoopAError = 30
        CurrentLoopActualCurrentA = 31
        CurrentLoopAIntegratorSum = 32
        CurrentLoopAIntegralContribution = 33
        CurrentLoopAOutput = 34
        CurrentLoopBReference = 67
        CurrentLoopBError = 35
        CurrentLoopActualCurrentB = 36
        CurrentLoopBIntegratorSum = 37
        CurrentLoopBIntegralContribution = 38
        CurrentLoopBOutput = 39
        FOCDReference = 40
        FOCDError = 41
        FOCDFeedback = 42
        FOCDIntegratorSum = 43
        FOCDIntegralContribution = 44
        FOCDOutput = 45
        FOCQReference = 46
        FOCQError = 47
        FOCQFeedback = 48
        FOCQIntegratorSum = 49
        FOCQIntegralContribution = 50
        FOCQOutput = 51
        FOCAOutput = 52
        FOCBOutput = 53
        FOCActualCurrentA = 31
        FOCActualCurrentB = 36
        BusVoltage = 54
        Temperature = 55
    End Enum

    Public Enum PMDTraceMode
        OneTime = 0
        RollingBuffer = 1
    End Enum

    Public Enum PMDTraceVariableNumber
        Variable1 = 0
        Variable2 = 1
        Variable3 = 2
        Variable4 = 3
    End Enum

    Public Enum PMDPhaseCommand
        A = 0
        B = 1
        C = 2
    End Enum

    Public Enum PMDPhasePrescaleMode
        PrescaleOff = 0
        Prescale64 = 1
        Prescale128 = 2
        Prescale256 = 3
    End Enum

    Public Enum PMDPhaseInitializeMode
        Algorithmic = 0
        HallBased = 1
    End Enum

    Public Enum PMDPhaseCorrectionMode
        Disable = 0
        Index = 1
        Hall = 2
    End Enum

    Public Enum PMDCommutationMode
        Sinusoidal = 0
        HallBased = 1
    End Enum

    Public Enum PMDMotorMode
        ModeOff = 0
        ModeOn = 1
    End Enum

    Public Enum PMDMotorOutputMode
        BipolarDAC = 0
        PWMSignMagnitude = 1
        PWM5050Magnitude = 2
        OffsetSPIDAC = 3
        UnipolarDAC = 4
        SPIDAC2sComplement = 5
        Atlas = 6
        PWMHighLow = 7
        PulseAndDirection = 8
        AtlasRecovery = 9
        None = 10
    End Enum

    Public Enum PMDEncoderSource
        Incremental = 0
        Parallel = 1
        None = 2
        Loopback = 3
        Hall = 5
        Parallel32 = 6
    End Enum

    Public Enum PMDCaptureSource
        Index = 0
        Home = 1
        HSI = 2
    End Enum

    Public Enum PMDSignalMask
        EncoderAMask = &H1&
        EncoderBMask = &H2&
        EncoderIndexMask = &H4&
        EncoderHomeMask = &H8&
        PositiveLimitMask = &H10&
        NegativeLimitMask = &H20&
        AxisInMask = &H40&
        HallAMask = &H80&
        HallBMask = &H100&
        HallCMask = &H200&
        AxisOutMask = &H400&
        StepOutputInvertMask = &H800&
        MotorDirectionMask = &H1000&
        EnableIn = &H2000&
        FaultOut = &H4000&
    End Enum

    Public Enum PMDEventMask
        MotionCompleteMask = &H1&
        WrapAroundMask = &H2&
        Breakpoint1Mask = &H4&
        CaptureReceivedMask = &H8&
        MotionErrorMask = &H10&
        InPositiveLimitMask = &H20&
        InNegativeLimitMask = &H40&
        InstructionErrorMask = &H80&
        DriveDisabledMask = &H100&
        OvertemperatureFaultMask = &H200&
        BusVoltageFaultMask = &H400&
        DriveExceptionMask = &H400&     'MC58113
        CommutationErrorMask = &H800&
        CurrentFoldbackMask = &H1000&
    End Enum

    Public Enum PMDActivityMask
        PhasingInitializedMask = &H1&
        AtMaximumVelocityMask = &H2&
        TrackingMask = &H4&
        ProfileModeMask = &H38&
        AxisSettledMask = &H80&
        MotorOnMask = &H100&
        PositionCaptureMask = &H200&
        InMotionMask = &H400&
        InPositiveLimitMask = &H800&
        InNegativeLimitMask = &H1000&
        ProfileSegmentMask = &HE000&
    End Enum

    Public Enum PMDActualPositionUnits
        Counts = 0
        Steps = 1
    End Enum

    Public Enum PMDBreakpointAction
        NoAction = 0
        Update = 1
        AbruptStop = 2
        SmoothStop = 3
        MotorOff = 4
        DisablePositionLoopAndHigherModules = 5
        DisableCurrentLoopAndHigherModules = 6
        DisableMotorOutputAndHigherModules = 7
        AbruptStopWithPositionErrorClear = 8
    End Enum

    Public Enum PMDBreakpointTrigger
        Disable = 0
        GreaterOrEqualCommandedPosition = 1
        LessOrEqualCommandedPosition = 2
        GreaterOrEqualActualPosition = 3
        LessOrEqualActualPosition = 4
        CommandedPositionCrossed = 5
        ActualPositionCrossed = 6
        Time = 7
        EventStatus = 8
        ActivityStatus = 9
        SignalStatus = 10
        DriveStatus = 11
    End Enum

    Public Enum PMDBreakpoint
        Breakpoint1 = 0
        Breakpoint2 = 1
    End Enum

    Public Enum PMDMotionCompleteMode
        CommandedPosition = 0
        ActualPosition = 1
    End Enum

    Public Enum PMDStopMode
        NoStop = 0
        Abrupt = 1
        Smooth = 2
    End Enum

    Public Enum PMDProfileMode
        Trapezoidal = 0
        VelocityContouring = 1
        SCurve = 2
        ElectronicGear = 3
    End Enum

    Public Enum PMDMotorType
        BrushlessDC_3Phase = 0
        BrushlessDC_2Phase = 1
        Microstepping_3Phase = 2
        Microstepping_2Phase = 3
        Stepper = 4
        DCBrush = 7
    End Enum

    Public Const AtlasAxisMask = &H20&

    Public Enum PMDAxisNumber
        Axis1 = 0
        Axis2 = 1
        Axis3 = 2
        Axis4 = 3
        AtlasAxis1 = Axis1 + AtlasAxisMask
        AtlasAxis2 = Axis2 + AtlasAxisMask
        AtlasAxis3 = Axis3 + AtlasAxisMask
        AtlasAxis4 = Axis4 + AtlasAxisMask
    End Enum

    Public Enum PMDAxisMask
        NoAxis = &H0&
        Axis1 = &H1&
        Axis2 = &H2&
        Axis3 = &H4&
        Axis4 = &H8&
    End Enum

    Public Enum PMDAuxiliaryEncoderMode
        Disable = 0
        Enable = 1
    End Enum

    Public Enum PMDGearMasterSource
        Actual = 0
        Commanded = 1
    End Enum

    Public Enum PMDFeedbackParameter
        EncoderModulus = 0
    End Enum

    Public Enum PMDresult
        NOERROR = &H0&
        ERR_OK = &H0&
        ERR_Reset = &H1&
        ERR_InvalidInstruction = &H2&
        ERR_InvalidAxis = &H3&
        ERR_InvalidParameter = &H4&
        ERR_TraceRunning = &H5&
        ERR_BlockOutOfBounds = &H7&
        ERR_TraceBufferZero = &H8&
        ERR_BadSerialChecksum = &H9&
        ERR_InvalidNegativeValue = &HB&
        ERR_InvalidParameterChange = &HC&
        ERR_LimitEventPending = &HD&
        ERR_InvalidMoveIntoLimit = &HE&
        ERR_InvalidOperatingModeRestore = &H10&
        ERR_InvalidOperatingModeForCommand = &H11&
        ERR_BadState = &H12&
        ERR_HardFault = &H13&
        ERR_AtlasNotDetected = &H14&
        ERR_BadSPIChecksum = &H15&
        ERR_InvalidSPIprotocol = &H16&
        ERR_InvalidTorqueCommand = &H18&
        ERR_BadFlashChecksum = &H19&
        ERR_InvalidFlashModeCommand = &H1A&
        ERR_ReadOnly = &H1B&
        ERR_InitializationOnlyCommand = &H1C&
        ERR_IncorrectDataCount = &H1D&
        ERR_Version = &H1002&
        ERR_Cancelled = &H1007&
        ERR_CommunicationsError = &H1008&
        ERR_InsufficientDataReceived = &H100A&
        ERR_UnexpectedDataReceived = &H100B&
        ERR_Memory = &H100C&
        ERR_Timeout = &H100D&
        ERR_Checksum = &H100E&
        ERR_CommandError = &H100F&
        ERR_NotSupported = &H1101&
        ERR_InvalidOperation = &H1102&
        ERR_InvalidInterface = &H1103&
        ERR_InvalidPort = &H1104&
        ERR_InvalidBaud = &H1105&
        ERR_InvalidHandle = &H1106&
        ERR_ParameterOutOfRange = &H110A&
        ERR_ParameterAlignment = &H110B&
        ERR_NotConnected = &H1201&
        ERR_NotResponding = &H1202&
        ERR_PortRead = &H1203&
        ERR_PortWrite = &H1204&
        ERR_OpeningPort = &H1205&
        ERR_ConfiguringPort = &H1206&
        ERR_InterfaceNotInitialized = &H1207&
        ERR_Driver = &H1208&
        ERR_AddressInUse = &H1209&
        ERR_IPRouting = &H120A&
        ERR_RP_Reset = &H2001&
        ERR_RP_InvalidVersion = &H2002&
        ERR_RP_InvalidResource = &H2003&
        ERR_RP_InvalidAddress = &H2004&
        ERR_RP_InvalidAction = &H2005&
        ERR_RP_InvalidSubAction = &H2006&
        ERR_RP_InvalidCommand = &H2007&
        ERR_RP_InvalidParameter = &H2008&
        ERR_RP_InvalidPacket = &H2009&
        ERR_RP_OutOfHandles = &H200A&
        ERR_RP_Checksum = &H200E&
        ERR_UC_Signature = &H2101&
        ERR_UC_Version = &H2102&
        ERR_UC_FileSize = &H2103&
        ERR_UC_Checksum = &H2104&
        ERR_UC_WriteError = &H2105&
        ERR_UC_NotProgrammed = &H2106&
        ERR_UC_TaskNotCreated = &H2107&
        ERR_UC_TaskAlreadyRunning = &H2108&
        ERR_UC_TaskNotFound = &H2109&
        ERR_UC_StartupCode = &H210A&
    End Enum

    '*** C library structure declarations

    ' Maintain two versions of this struct so our interface can be CLS compliant.
    <StructLayout(LayoutKind.Sequential)> Friend Structure PMDEvent_internal
        <MarshalAs(UnmanagedType.U2)> Public axis As PMDAxisNumber
        Public EventMask As UInt16
    End Structure

    Public Structure PMDEvent
        Public axis As PMDAxisNumber
        Public EventMask As UInt16
    End Structure

    '*** DLL entry point declarations ***

    Friend Declare Function PMDAdjustActualPosition Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal position As Int32) As PMDresult
    Friend Declare Function PMDAxisAlloc Lib "C-Motion.dll" () As IntPtr
    Friend Declare Function PMDAxisClose Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Sub PMDAxisFree Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr)
    Friend Declare Function PMDAxisOpen Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal hDev As IntPtr, _
                            ByVal axis_number As UInt16) As PMDresult
    Friend Declare Function PMDAtlasAxisOpen Lib "C-Motion.dll" ( _
                            ByVal hSourceAxis As IntPtr, _
                            ByVal hAtlasAxis As IntPtr) As PMDresult
    Friend Declare Function PMDTaskGetState Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr, _
                            ByRef state As PMDTaskState) As PMDresult
    Friend Declare Function PMDTaskStart Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr) As PMDresult
    Friend Declare Function PMDTaskStop Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr) As PMDresult
    Friend Declare Function PMDCalibrateAnalog Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16) As PMDresult
    Friend Declare Function PMDClearDriveFaultStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Function PMDClearInterrupt Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Function PMDClearPositionError Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Function PMDDeviceAlloc Lib "C-Motion.dll" () As IntPtr
    Friend Declare Function PMDDeviceClose Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr) As PMDresult
    Friend Declare Sub PMDDeviceFree Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr)

    ' The C function uses a pointer to void for value
    Friend Declare Function PMDDeviceGetDefaultUInt32 Lib "C-Motion.dll" Alias "PMDDeviceGetDefault" ( _
                            ByVal hDev As IntPtr, _
                            ByVal code As PMDDefault, _
                            ByRef value As UInt32, _
                            ByVal ValueSize As UInt32) As PMDresult
    Friend Declare Function PMDDeviceGetVersion Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr, _
                            ByRef major As UInt32, _
                            ByRef minor As UInt32) As PMDresult
    Friend Declare Function PMDDeviceReset Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr) As PMDresult
    Friend Declare Function PMDDeviceSetDefaultUInt32 Lib "C-Motion.dll" Alias "PMDDeviceSetDefault" ( _
                            ByVal hDev As IntPtr, _
                            ByVal code As PMDDefault, _
                            ByRef value As UInt32, _
                            ByVal ValueSize As UInt32) As PMDresult
    Friend Declare Function PMDDriveNVRAM Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByVal value As UInt16) As PMDresult
    Friend Declare Function PMDGetAcceleration Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef acceleration As UInt32) As PMDresult
    Friend Declare Function PMDGetActiveMotorCommand Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef command As Int16) As PMDresult
    Friend Declare Function PMDGetActiveOperatingMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef mode As UInt16) As PMDresult
    Friend Declare Function PMDGetActivityStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef status As UInt16) As PMDresult
    Friend Declare Function PMDGetActualPosition Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef position As Int32) As PMDresult
    Friend Declare Function PMDGetActualPositionUnits Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetActualVelocity Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetAnalogCalibration Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal channel As Int16, _
                            ByRef value As Int16) As PMDresult
    Friend Declare Function PMDSetAnalogCalibration Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal channel As Int16, _
                            ByVal value As Int16) As PMDresult
    Friend Declare Function PMDGetAuxiliaryEncoderSource Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef mode As Byte, _
                            ByRef AuxAxis As UInt16) As PMDresult
    Friend Declare Function PMDGetAxisOutMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef SourceAxis As UInt16, _
                            ByRef SourceRegister As Byte, _
                            ByRef SelectionMask As UInt16, _
                            ByRef SenseMask As UInt16) As PMDresult
    Friend Declare Function PMDGetBreakpoint Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal BreakpointId As UInt16, _
                            ByRef SourceAxis As UInt16, _
                            ByRef Action As Byte, _
                            ByRef Trigger As Byte) As PMDresult
    Friend Declare Function PMDGetBreakpointUpdateMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDGetBreakpointValue Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As Int32) As PMDresult
    Friend Declare Function PMDGetBufferLength Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal BufferId As UInt16, _
                            ByRef ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDGetBufferReadIndex Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal BufferId As UInt16, _
                            ByRef ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDGetBufferStart Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal BufferId As UInt16, _
                            ByRef ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDGetBufferWriteIndex Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal BufferId As UInt16, _
                            ByRef ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDGetBusVoltage Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetBusVoltageLimits Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByRef ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDGetCANMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef NodeId As Byte, _
                            ByRef TransmissionRate As Byte) As PMDresult
    Friend Declare Sub PMDGetCMotionVersion Lib "C-Motion.dll" ( _
                            ByRef ARG1 As UInt32, _
                            ByRef ARG2 As UInt32)
    Friend Declare Function PMDGetCaptureSource Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetCaptureValue Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetChecksum Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetCommandedAcceleration Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetCommandedPosition Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetCommandedVelocity Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetCommutationMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetCurrent Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByRef value As UInt16) As PMDresult
    Friend Declare Function PMDGetCurrentControlMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetCurrentFoldback Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByRef value As UInt16) As PMDresult
    Friend Declare Function PMDGetCurrentLoop Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal phase As Byte, _
                            ByVal parameter As Byte, _
                            ByRef value As UInt16) As PMDresult
    Friend Declare Function PMDGetCurrentLoopValue Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Byte, _
                            ByVal ARG3 As Byte, _
                            ByRef ARG4 As Int32) As PMDresult
    Friend Declare Function PMDGetDeceleration Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetDefault Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDGetDriveFaultStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetDriveCommandMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef transport As Byte, _
                            ByRef format As Byte) As PMDresult
    Friend Declare Function PMDGetDriveFaultParameter Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByRef value As UInt16) As PMDresult
    Friend Declare Function PMDGetDrivePWM Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByRef value As UInt16) As PMDresult
    Friend Declare Function PMDGetDriveStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetDriveValue Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal node As UInt16, _
                            ByRef value As UInt16) As PMDresult
    Friend Declare Function PMDGetEncoderModulus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetEncoderSource Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetEncoderToStepRatio Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef counts As UInt16, _
                            ByRef steps As UInt16) As PMDresult
    Friend Declare Function PMDGetEventAction Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDGetEventStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetFeedbackParameter Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByRef value As UInt32) As PMDresult
    Friend Declare Function PMDGetFOC Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Byte, _
                            ByVal ARG3 As Byte, _
                            ByRef ARG4 As UInt16) As PMDresult
    Friend Declare Function PMDGetFOCValue Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Byte, _
                            ByVal ARG3 As Byte, _
                            ByRef ARG4 As Int32) As PMDresult
    Friend Declare Function PMDGetFaultOutMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetGearMaster Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef MasterAxis As UInt16, _
                            ByRef source As UInt16) As PMDresult
    Friend Declare Function PMDGetGearRatio Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetHoldingCurrent Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDGetInstructionError Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetInterruptAxis Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetInterruptMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetJerk Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetMotionCompleteMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetMotorBias Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int16) As PMDresult
    Friend Declare Function PMDGetMotorCommand Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int16) As PMDresult
    Friend Declare Function PMDGetMotorLimit Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetMotorType Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetOperatingMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetOutputMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetOvertemperatureLimit Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPWMFrequency Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPhaseAngle Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPhaseCommand Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As Int16) As PMDresult
    Friend Declare Function PMDGetPhaseCorrectionMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPhaseCounts Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPhaseInitializeMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPhaseInitializeTime Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPhaseOffset Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPhasePrescale Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetPosition Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetPositionError Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetPositionErrorLimit Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetPositionLoop Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As Int32) As PMDresult
    Friend Declare Function PMDGetPositionLoopValue Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As Int32) As PMDresult
    Friend Declare Function PMDGetProfileMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetSPIMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetSampleTime Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetSerialPortMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef baud As Byte, _
                            ByRef parity As Byte, _
                            ByRef StopBits As Byte, _
                            ByRef protocol As Byte, _
                            ByRef MultiDropId As Byte) As PMDresult
    Friend Declare Function PMDGetSettleTime Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetSettleWindow Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetSignalSense Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetSignalStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetStartVelocity Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetStepRange Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetStopMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetSynchronizationMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetTemperature Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetTime Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetTraceCount Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDGetTraceMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetTracePeriod Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetTraceStart Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16, _
                            ByRef ARG3 As Byte, _
                            ByRef ARG4 As Byte, _
                            ByRef ARG5 As Byte) As PMDresult
    Friend Declare Function PMDGetTraceStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetTraceStop Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16, _
                            ByRef ARG3 As Byte, _
                            ByRef ARG4 As Byte, _
                            ByRef ARG5 As Byte) As PMDresult
    Friend Declare Function PMDGetTraceVariable Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As UInt16, _
                            ByRef ARG4 As Byte) As PMDresult
    Friend Declare Function PMDGetTrackingWindow Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetUpdateMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDGetVelocity Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef ARG2 As Int32) As PMDresult
    Friend Declare Function PMDGetVersion Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByRef family As UInt16, _
                            ByRef motorType As UInt16, _
                            ByRef numberAxes As UInt16, _
                            ByRef special_and_chip_count As UInt16, _
                            ByRef custom As UInt16, _
                            ByRef major As UInt16, _
                            ByRef minor As UInt16) As PMDresult
    Friend Declare Function PMDInitializePhase Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Function PMDMPDeviceOpen Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr, _
                            ByVal hPeriph As IntPtr) As PMDresult
    Friend Declare Function PMDMemoryAlloc Lib "C-Motion.dll" () As IntPtr
    Friend Declare Function PMDMemoryClose Lib "C-Motion.dll" ( _
                            ByVal hMem As IntPtr) As PMDresult
    Friend Declare Sub PMDMemoryFree Lib "C-Motion.dll" ( _
                            ByVal hMem As IntPtr)
    Friend Declare Function PMDMemoryOpen Lib "C-Motion.dll" ( _
                            ByVal hMem As IntPtr, _
                            ByVal hDev As IntPtr, _
                            ByVal DataSize As PMDDataSize, _
                            ByVal MemType As PMDMemoryType) As PMDresult

    ' Note that the ByRef data argument is the first element of an array.
    Friend Declare Function PMDMemoryRead Lib "C-Motion.dll" ( _
                            ByVal hMem As IntPtr, _
                            ByRef data As UInt32, _
                            ByVal offset As UInt32, _
                            ByVal length As UInt32) As PMDresult

    'Note that the ByRef data argument is the first element of an array
    Friend Declare Function PMDMemoryWrite Lib "C-Motion.dll" ( _
                            ByVal hMem As IntPtr, _
                            ByRef data As UInt32, _
                            ByVal offset As UInt32, _
                            ByVal length As UInt32) As PMDresult
    Friend Declare Function PMDMultiUpdate Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDNoOperation Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Function PMDPeriphAlloc Lib "C-Motion.dll" () As IntPtr
    Friend Declare Function PMDPeriphClose Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr) As PMDresult
    Friend Declare Function PMDPeriphFlush Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr) As PMDresult
    Friend Declare Sub PMDPeriphFree Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr)
    Friend Declare Function PMDPeriphOpenCAN Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByVal hDev As IntPtr, _
                            ByVal transmitid As UInt32, _
                            ByVal receiveid As UInt32, _
                            ByVal eventid As UInt32) As PMDresult
    Friend Declare Function PMDPeriphOpenCME Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByVal hDev As IntPtr) As PMDresult
    Friend Declare Function PMDPeriphOpenCOM Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByVal hDev As IntPtr, _
                            ByVal portnum As UInt32, _
                            ByVal baud As PMDSerialBaud, _
                            ByVal parity As PMDSerialParity, _
                            ByVal StopBits As PMDSerialStopBits) As PMDresult
    Friend Declare Function PMDPeriphOpenMultiDrop Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByVal hPeriph As IntPtr, _
                            ByVal ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDPeriphOpenPCI Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByVal ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDPeriphOpenTCP Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByVal hDev As IntPtr, _
                            ByVal IPAddress As UInt32, _
                            ByVal portnum As UInt32, _
                            ByVal timeout As UInt32) As PMDresult
    Friend Declare Function PMDPeriphOpenPIO Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByVal hDev As IntPtr, _
                            ByVal address As UInt16, _
                            ByVal eventIRQ As Byte, _
                            ByVal datasize As PMDDataSize) As PMDresult


    ' These functions are for reading and writing 8 or 16 bit data over a parallel bus peripheral.
    ' The width of the data has to be correct for the peripheral type.
    ' The data argument should be the first member of an array of Byte or UInt16.
    Friend Declare Function PMDPeriphReadUInt8 Lib "C-Motion.dll" Alias "PMDPeriphRead" ( _
                            ByVal hPeriph As IntPtr, _
                            ByRef data As Byte, _
                            ByVal offset As UInt32, _
                            ByVal length As UInt32) As PMDresult

    Friend Declare Function PMDPeriphReadUInt16 Lib "C-Motion.dll" Alias "PMDPeriphRead" ( _
                            ByVal hPeriph As IntPtr, _
                            ByRef data As UInt16, _
                            ByVal offset As UInt32, _
                            ByVal length As UInt32) As PMDresult

    Friend Declare Function PMDPeriphWriteUInt8 Lib "C-Motion.dll" Alias "PMDPeriphWrite" ( _
                            ByVal hPeriph As IntPtr, _
                            ByRef data As Byte, _
                            ByVal offset As UInt32, _
                            ByVal length As UInt32) As PMDresult

    Friend Declare Function PMDPeriphWriteUInt16 Lib "C-Motion.dll" Alias "PMDPeriphWrite" ( _
                            ByVal hPeriph As IntPtr, _
                            ByRef data As UInt16, _
                            ByVal offset As UInt32, _
                            ByVal length As UInt32) As PMDresult

    ' data should be an array of unsigned bytes, the first element should be passed.
    ' strings could be supported as well.
    Friend Declare Function PMDPeriphReceive Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByRef data As Byte, _
                            ByRef nReceived As UInt32, _
                            ByVal nExpected As UInt32, _
                            ByVal timeout As UInt32) As PMDresult

    ' data should be an array of unsigned bytes, the first element should be passed.
    ' strings could be supported as well.
    Friend Declare Function PMDPeriphSend Lib "C-Motion.dll" ( _
                            ByVal hPeriph As IntPtr, _
                            ByRef data As Byte, _
                            ByVal nCount As UInt32, _
                            ByVal timeout As UInt32) As PMDresult

    Friend Declare Function PMDRPDeviceOpen Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr, _
                            ByVal hPeriph As IntPtr) As PMDresult
    Friend Declare Function PMDReadAnalog Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDReadBuffer Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal bufferID As UInt16, _
                            ByRef data As Int32) As PMDresult
    Friend Declare Function PMDReadBuffer16 Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal bufferID As UInt16, _
                            ByRef data As Int16) As PMDresult
    Friend Declare Function PMDReadIO Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByRef ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDReset Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Function PMDResetEventStatus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDRestoreOperatingMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult
    Friend Declare Function PMDSetAcceleration Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDSetActualPosition Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Int32) As PMDresult
    Friend Declare Function PMDSetActualPositionUnits Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetAuxiliaryEncoderSource Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Byte, _
                            ByVal ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDSetAxisOutMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As Byte, _
                            ByVal ARG4 As UInt16, _
                            ByVal ARG5 As UInt16) As PMDresult
    Friend Declare Function PMDSetBreakpoint Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt16, _
                            ByVal ARG4 As Byte, _
                            ByVal ARG5 As Byte) As PMDresult
    Friend Declare Function PMDSetBreakpointUpdateMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDSetBreakpointValue Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As Int32) As PMDresult
    Friend Declare Function PMDSetBufferLength Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDSetBufferReadIndex Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDSetBufferStart Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDSetBufferWriteIndex Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDSetBusVoltageLimits Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDSetCANMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Byte, _
                            ByVal ARG3 As Byte) As PMDresult
    Friend Declare Function PMDSetCaptureSource Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetCommutationMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetCurrent Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByVal value As UInt16) As PMDresult
    Friend Declare Function PMDSetCurrentControlMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetCurrentFoldback Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByVal value As UInt16) As PMDresult
    Friend Declare Function PMDSetCurrentLoop Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal phase As Byte, _
                            ByVal parameter As Byte, _
                            ByVal value As UInt16) As PMDresult
    Friend Declare Function PMDSetDeceleration Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDSetDefault Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt32) As PMDresult
    Friend Declare Function PMDSetDriveCommandMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal transport As Byte, _
                            ByVal format As Byte) As PMDresult
    Friend Declare Function PMDSetDriveFaultParameter Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByVal value As UInt16) As PMDresult
    Friend Declare Function PMDSetDrivePWM Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByVal value As UInt16) As PMDresult
    Friend Declare Function PMDSetEncoderModulus Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetEncoderSource Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetEncoderToStepRatio Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal counts As UInt16, _
                            ByVal steps As UInt16) As PMDresult
    Friend Declare Function PMDSetEventAction Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDSetFeedbackParameter Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal parameter As UInt16, _
                            ByVal value As UInt32) As PMDresult
    Friend Declare Function PMDSetFOC Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Byte, _
                            ByVal ARG3 As Byte, _
                            ByVal ARG4 As UInt16) As PMDresult
    Friend Declare Function PMDSetFaultOutMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetGearMaster Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal MasterAxis As UInt16, _
                            ByVal source As UInt16) As PMDresult
    Friend Declare Function PMDSetGearRatio Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Int32) As PMDresult
    Friend Declare Function PMDSetHoldingCurrent Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt16) As PMDresult
    Friend Declare Function PMDSetInterruptMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetJerk Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDSetMotionCompleteMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetMotorBias Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Int16) As PMDresult
    Friend Declare Function PMDSetMotorCommand Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Int16) As PMDresult
    Friend Declare Function PMDSetMotorLimit Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetMotorType Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetOperatingMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetOutputMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetOvertemperatureLimit Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPWMFrequency Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPhaseAngle Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPhaseCorrectionMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPhaseCounts Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPhaseInitializeMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPhaseInitializeTime Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPhaseOffset Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPhasePrescale Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetPosition Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Int32) As PMDresult
    Friend Declare Function PMDSetPositionErrorLimit Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDSetPositionLoop Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As Int32) As PMDresult
    Friend Declare Function PMDSetProfileMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetSPIMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetSampleTime Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDSetSerialPortMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Byte, _
                            ByVal ARG3 As Byte, _
                            ByVal ARG4 As Byte, _
                            ByVal ARG5 As Byte, _
                            ByVal ARG6 As Byte) As PMDresult
    Friend Declare Function PMDSetSettleTime Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetSettleWindow Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetSignalSense Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetStartVelocity Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt32) As PMDresult
    Friend Declare Function PMDSetStepRange Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetStopMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetSynchronizationMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetTraceMode Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetTracePeriod Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetTraceStart Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As Byte, _
                            ByVal ARG4 As Byte, _
                            ByVal ARG5 As Byte) As PMDresult
    Friend Declare Function PMDSetTraceStop Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As Byte, _
                            ByVal ARG4 As Byte, _
                            ByVal ARG5 As Byte) As PMDresult
    Friend Declare Function PMDSetTraceVariable Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt16, _
                            ByVal ARG4 As Byte) As PMDresult
    Friend Declare Function PMDSetTrackingWindow Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetUpdateMask Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16) As PMDresult
    Friend Declare Function PMDSetVelocity Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As Int32) As PMDresult
    Friend Declare Function PMDUpdate Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr) As PMDresult

    Friend Declare Function PMDWaitForEvent Lib "C-Motion.dll" ( _
                            ByVal hDev As IntPtr, _
                            ByRef EventStruct As PMDEvent_internal, _
                            ByVal timeout As UInt32) As PMDresult

    Friend Declare Function PMDWriteBuffer Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As Int32) As PMDresult
    Friend Declare Function PMDWriteIO Lib "C-Motion.dll" ( _
                            ByVal hAxis As IntPtr, _
                            ByVal ARG2 As UInt16, _
                            ByVal ARG3 As UInt16) As PMDresult

    '*** Utility functions
    Public Function PMDresultString(ByVal status As PMDresult) As String
        Select Case status
            Case PMDresult.NOERROR
                Return "0: No error"
            Case PMDresult.ERR_Reset
                Return "1: Reset"
            Case PMDresult.ERR_InvalidInstruction
                Return "2: Invalid Instruction"
            Case PMDresult.ERR_InvalidAxis
                Return "3: Invalid Axis"
            Case PMDresult.ERR_InvalidParameter
                Return "4: Invalid Parameter"
            Case PMDresult.ERR_TraceRunning
                Return "5: Trace Running"
            Case PMDresult.ERR_BlockOutOfBounds
                Return "7: Block Out Of Bounds"
            Case PMDresult.ERR_TraceBufferZero
                Return "8: Trace Buffer Zero"
            Case PMDresult.ERR_BadSerialChecksum
                Return "9: Bad Serial Checksum"
            Case PMDresult.ERR_InvalidNegativeValue
                Return "11: Invalid Negative Value"
            Case PMDresult.ERR_InvalidParameterChange
                Return "12: Invalid Parameter Change"
            Case PMDresult.ERR_LimitEventPending
                Return "13: Limit Event Pending"
            Case PMDresult.ERR_InvalidMoveIntoLimit
                Return "14: Invalid Move Into Limit"
            Case PMDresult.ERR_InvalidOperatingModeRestore
                Return "16: Invalid Operating Mode Restore"
            Case PMDresult.ERR_InvalidOperatingModeForCommand
                Return "17: Invalid Operating Mode For Command"
            Case PMDresult.ERR_BadState
                Return "18: BadState"
            Case PMDresult.ERR_HardFault
                Return "19: HardFault"
            Case PMDresult.ERR_AtlasNotDetected
                Return "20: AtlasNotDetected"
            Case PMDresult.ERR_BadSPIChecksum
                Return "21: BadSPIChecksum"
            Case PMDresult.ERR_InvalidSPIprotocol
                Return "22: InvalidSPIprotocol"
            Case PMDresult.ERR_InvalidTorqueCommand
                Return "24: InvalidTorqueCommand"
            Case PMDresult.ERR_BadFlashChecksum
                Return "25: BadFlashChecksum"
            Case PMDresult.ERR_InvalidFlashModeCommand
                Return "26: InvalidFlashModeCommand"
            Case PMDresult.ERR_ReadOnly
                Return "27: ReadOnly"
            Case PMDresult.ERR_InitializationOnlyCommand
                Return "28: InitializationOnlyCommand"
            Case PMDresult.ERR_Version
                Return "4098: Version"
            Case PMDresult.ERR_Cancelled
                Return "4103: Cancelled"
            Case PMDresult.ERR_CommunicationsError
                Return "4104: Communications Error"
            Case PMDresult.ERR_InsufficientDataReceived
                Return "4106: Insufficient Data Received"
            Case PMDresult.ERR_UnexpectedDataReceived
                Return "4107: Unexpected Data Received"
            Case PMDresult.ERR_Memory
                Return "4108: Memory"
            Case PMDresult.ERR_Timeout
                Return "4109: Timeout"
            Case PMDresult.ERR_Checksum
                Return "4110: Checksum"
            Case PMDresult.ERR_CommandError
                Return "4111: Command Error"
            Case PMDresult.ERR_NotSupported
                Return "4353: Not Supported"
            Case PMDresult.ERR_InvalidOperation
                Return "4354: Invalid Operation"
            Case PMDresult.ERR_InvalidInterface
                Return "4355: Invalid Interface"
            Case PMDresult.ERR_InvalidPort
                Return "4356: Invalid Port"
            Case PMDresult.ERR_InvalidBaud
                Return "4357: Invalid Baud"
            Case PMDresult.ERR_InvalidHandle
                Return "4358: Invalid Handle"
            Case PMDresult.ERR_ParameterOutOfRange
                Return "4362: Parameter Out Of Range"
            Case PMDresult.ERR_ParameterAlignment
                Return "4363: Parameter Alignment"
            Case PMDresult.ERR_NotConnected
                Return "4609: Not Connected"
            Case PMDresult.ERR_NotResponding
                Return "4610: Not Responding"
            Case PMDresult.ERR_PortRead
                Return "4611: Port Read"
            Case PMDresult.ERR_PortWrite
                Return "4612: Port Write"
            Case PMDresult.ERR_OpeningPort
                Return "4613: Opening Port"
            Case PMDresult.ERR_ConfiguringPort
                Return "4614: Configuring Port"
            Case PMDresult.ERR_InterfaceNotInitialized
                Return "4615: Interface Not Initialized"
            Case PMDresult.ERR_Driver
                Return "4616: Driver"
            Case PMDresult.ERR_AddressInUse
                Return "4617: Address In Use"
            Case PMDresult.ERR_IPRouting
                Return "4618: IPRouting"
            Case PMDresult.ERR_RP_Reset
                Return "8193: RP Reset"
            Case PMDresult.ERR_RP_InvalidVersion
                Return "8194: RP Invalid Version"
            Case PMDresult.ERR_RP_InvalidResource
                Return "8195: RP Invalid Resource"
            Case PMDresult.ERR_RP_InvalidAddress
                Return "8196: RP Invalid Address"
            Case PMDresult.ERR_RP_InvalidAction
                Return "8197: RP Invalid Action"
            Case PMDresult.ERR_RP_InvalidSubAction
                Return "8198: RP Invalid Sub Action"
            Case PMDresult.ERR_RP_InvalidCommand
                Return "8199: RP Invalid Command"
            Case PMDresult.ERR_RP_InvalidParameter
                Return "8200: RP Invalid Parameter"
            Case PMDresult.ERR_RP_InvalidPacket
                Return "8201: RP Invalid Packet"
            Case PMDresult.ERR_RP_OutOfHandles
                Return "8202: RP Out Of Handles"
            Case PMDresult.ERR_RP_Checksum
                Return "8206: RP Checksum"
            Case PMDresult.ERR_UC_Signature
                Return "8449: UC Signature"
            Case PMDresult.ERR_UC_Version
                Return "8450: UC Version"
            Case PMDresult.ERR_UC_FileSize
                Return "8451: UC File Size"
            Case PMDresult.ERR_UC_Checksum
                Return "8452: UC Checksum"
            Case PMDresult.ERR_UC_WriteError
                Return "8453: UC Write Error"
            Case PMDresult.ERR_UC_NotProgrammed
                Return "8454: UC Not Programmed"
            Case PMDresult.ERR_UC_TaskNotCreated
                Return "8455: UC Task Not Created"
            Case PMDresult.ERR_UC_TaskAlreadyRunning
                Return "8456: UC Task Already Running"
            Case PMDresult.ERR_UC_TaskNotFound
                Return "8457: UC Task Not Found"
            Case PMDresult.ERR_UC_StartupCode
                Return "8458: UC Startup Code"
            Case Else
                Return status.toString() & ": ??"
        End Select
    End Function


    Public Class PMDPeripheral
        '*** Private data and utility functions ***
        'hPeriph is a pointer to the C-allocated peripheral handle, it is not user-visible in VB
        Friend hPeriph As IntPtr
        Friend status As PMDresult

        Friend Sub CheckResult(ByVal status As PMDresult)
            Me.status = status
            If (Not status = PMDresult.NOERROR) Then
                Dim e As New Exception("ERROR: PMDPeripheral " & PMDresultString(status))
                e.Data.Add("PMDresult", status)
                Throw e
            End If
        End Sub

        '*** Public Methods ***
        Public Sub New()
            hPeriph = PMDPeriphAlloc()
            If (hPeriph = 0) Then
                Throw New Exception("ERROR: PMD library: could not allocate peripheral object")
            End If
        End Sub

        Protected Overrides Sub Finalize()
            If (Not hPeriph = 0) Then
                PMDPeriphClose(hPeriph)
                PMDPeriphFree(hPeriph)
                hPeriph = 0
            End If
        End Sub

        Public Sub Close()
            Me.Finalize()
        End Sub

        Public ReadOnly Property LastError() As PMDresult
            Get
                Return status
            End Get
        End Property

        Public Sub Flush()
            CheckResult(PMDPeriphFlush(hPeriph))
        End Sub

        Public Sub Send(ByRef data() As Byte, ByVal nCount As UInt32, ByVal timeout As UInt32)
            If (nCount > data.Length()) Then
                Throw New Exception("PMDPeripheral.Send bad nCount")
            End If
            CheckResult(PMDPeriphSend(hPeriph, data(0), nCount, timeout))
        End Sub

        Public Sub Receive(ByRef data() As Byte, ByRef nReceived As UInt32, _
                           ByVal nExpected As UInt32, ByVal timeout As UInt32)
            Dim nrecv As UInt32
            If (nExpected > data.Length()) Then
                Throw New Exception("PMDPeripheral.Receive bad nExpected")
            End If
            CheckResult(PMDPeriphReceive(hPeriph, data(0), nrecv, nExpected, timeout))
            nReceived = nrecv
        End Sub
        Public Sub Read(ByRef data As Byte(), ByVal offset As UInt32, ByVal length As UInt32)
            If (length > data.Length()) Then
                Throw New Exception("PMDPeripheral.Read bad length")
            End If
            CheckResult(PMDPeriphReadUInt8(hPeriph, data(0), offset, length))
        End Sub

        Public Sub Read(ByRef data As UInt16(), ByVal offset As UInt32, ByVal length As UInt32)
            If (length > data.Length()) Then
                Throw New Exception("PMDPeripheral.Read bad length")
            End If
            CheckResult(PMDPeriphReadUInt16(hPeriph, data(0), offset, length))
        End Sub

        Public Function Read(ByVal offset As UInt32) As UInt16
            Dim data As UInt16
            CheckResult(PMDPeriphReadUInt16(hPeriph, data, offset, 1))
            Return data
        End Function

        Public Sub Write(ByRef data As Byte(), ByVal offset As UInt32, ByVal length As UInt32)
            If (length > data.Length()) Then
                Throw New Exception("PMDPeripheral.Write bad length")
            End If
            CheckResult(PMDPeriphWriteUInt8(hPeriph, data(0), offset, length))
        End Sub

        Public Sub Write(ByRef data As UInt16(), ByVal offset As UInt32, ByVal length As UInt32)
            If (length > data.Length()) Then
                Throw New Exception("PMDPeripheral.Write bad length")
            End If
            CheckResult(PMDPeriphWriteUInt16(hPeriph, data(0), offset, length))
        End Sub

        Public Sub Write(ByVal data As UInt16, ByVal offset As UInt32)
            CheckResult(PMDPeriphWriteUInt16(hPeriph, data, offset, 1))
        End Sub

    End Class    'PMDPeripheral

    Public Class PMDPeripheralCAN
        Inherits PMDPeripheral

        Public Sub New(ByVal transmitid As UInt32, ByVal receiveid As UInt32, ByVal eventid As UInt32)
            MyBase.New()
            Dim r As PMDresult
            r = PMDPeriphOpenCAN(hPeriph, 0, transmitid, receiveid, eventid)
            If (Not r = PMDresult.NOERROR) Then
                MyBase.Close()
                CheckResult(r)
            End If
        End Sub

    End Class   'PMDPeripheralCAN

    Public Class PMDPeripheralCOM
        Inherits PMDPeripheral

        Public Sub New(ByVal portnum As UInt32, ByVal baud As PMDSerialBaud, _
                       ByVal parity As PMDSerialParity, ByVal StopBits As PMDSerialStopBits)
            MyBase.New()
            Dim r As PMDresult
            r = PMDPeriphOpenCOM(hPeriph, 0, portnum, baud, parity, StopBits)
            If (Not r = PMDresult.NOERROR) Then
                MyBase.Close()
                CheckResult(r)
            End If
        End Sub

    End Class   'PMDPeripheralCOM

    Public Class PMDPeripheralMultiDrop
        Inherits PMDPeripheral

        Public Sub New(ByRef parent As PMDPeripheral, ByVal address As Int16)
            MyBase.New()
            Dim r As PMDresult
            r = PMDPeriphOpenMultiDrop(hPeriph, parent.hPeriph, address)
            If (Not r = PMDresult.NOERROR) Then
                MyBase.Close()
                CheckResult(r)
            End If
        End Sub

    End Class   'PMDPeripheralMultiDrop

    Public Class PMDPeripheralCME
        Inherits PMDPeripheral

        Public Sub New(ByRef device As PMDDevice)
            MyBase.New()
            Dim r As PMDresult
            r = PMDPeriphOpenCME(hPeriph, device.hDev)
            If (Not r = PMDresult.NOERROR) Then
                MyBase.Close()
                CheckResult(r)
            End If
        End Sub

    End Class   'PMDPeripheralCME

    Public Class PMDPeripheralPCI
        Inherits PMDPeripheral

        Public Sub New(ByVal boardnum As Int16)
            MyBase.New()
            Dim r As PMDresult
            r = PMDPeriphOpenPCI(hPeriph, boardnum)
            If (Not r = PMDresult.NOERROR) Then
                MyBase.Close()
                CheckResult(r)
            End If
        End Sub

    End Class   'PMDPeripheralPCI

    Public Class PMDPeripheralTCP
        Inherits PMDPeripheral

        Public Sub New(ByVal address As System.Net.IPAddress, _
                       ByVal portnum As UInt32, _
                       ByVal timeout As UInt32)
            MyBase.New()
            Dim r As PMDresult
            Dim uaddr As UInt32
            Dim addr_bytes As Byte()
            Dim i As Integer
            If (Not (address.AddressFamily.Equals(System.Net.Sockets.AddressFamily.InterNetwork))) Then
                Throw New Exception("PMDPeripheralTCP supports only IPV4")
            End If

            ' High byte is first, low byte is last
            addr_bytes = address.GetAddressBytes()
            uaddr = 0
            For i = 0 To addr_bytes.Length - 1
                uaddr = (uaddr << 8) + addr_bytes(i)
            Next i

            r = PMDPeriphOpenTCP(hPeriph, 0, uaddr, portnum, timeout)
            If (Not r = PMDresult.NOERROR) Then
                MyBase.Close()
                CheckResult(r)
            End If
        End Sub

    End Class   'PMDPeripheralTCP

    Public Class PMDPeripheralPIO
        Inherits PMDPeripheral

        Public Sub New(ByVal device As PMDDevice,
                       ByVal address As Int16, _
                       ByVal eventIRQ As Byte, _
                       ByVal datasize As PMDDataSize)
            MyBase.New()
            Dim r As PMDresult
            r = PMDPeriphOpenPIO(hPeriph, device.hDev, address, eventIRQ, datasize)
            If (Not r = PMDresult.NOERROR) Then
                MyBase.Close()
                CheckResult(r)
            End If
        End Sub
    End Class
    Public Class PMDDevice
        '*** Private data and utility functions ***
        Friend hDev As IntPtr
        Public DeviceType As PMDDeviceType
        Friend status As PMDresult

        Friend Sub CheckResult(ByVal status As PMDresult)
            Me.status = status
            If (Not status = PMDresult.NOERROR) Then
                Dim e As New Exception("ERROR: PMDDevice " & PMDresultString(status))
                e.Data.Add("PMDresult", status)
                Throw e
            End If
        End Sub

        '*** Public Methods ***
        Public Sub New(ByVal periph As PMDPeripheral, ByVal dtype As PMDDeviceType)
            hDev = PMDDeviceAlloc()
            If (hDev = 0) Then
                Throw New Exception("ERROR: PMD library: could not allocate device object")
            End If

            Select Case dtype
                Case PMDDeviceType.MotionProcessor
                    CheckResult(PMDMPDeviceOpen(hDev, periph.hPeriph))
                Case PMDDeviceType.ResourceProtocol
                    CheckResult(PMDRPDeviceOpen(hDev, periph.hPeriph))

                    '*** Check for ERR_RP_Reset
                    'Remove the following section if you want to be informed of unexpected resets
                    Dim major As UInteger
                    Dim minor As UInteger
                    Me.status = PMDDeviceGetVersion(Me.hDev, major, minor)
                    If (PMDresult.ERR_RP_Reset = Me.status) Then
                        Me.status = PMDDeviceGetVersion(Me.hDev, major, minor)
                    End If
                    CheckResult(Me.status)
                    'End check for ERR_RP_Reset
            End Select
            DeviceType = dtype
        End Sub

        Protected Overrides Sub Finalize()
            If (Not hDev = 0) Then
                PMDDeviceClose(hDev)
                PMDDeviceFree(hDev)
                hDev = 0
            End If
        End Sub

        Public Sub Close()
            Me.Finalize()
        End Sub

        Public ReadOnly Property LastError() As PMDresult
            Get
                Return status
            End Get
        End Property

        Public Sub Reset()
            PMDDeviceReset(hDev)
        End Sub

        Public Sub Version(ByRef major As UInt32, ByRef minor As UInt32)
            CheckResult(PMDDeviceGetVersion(hDev, major, minor))
        End Sub

        Public Function WaitForEvent(ByRef EventStruct As PMDEvent, _
                                     ByVal timeout As UInt32) As PMDresult
            Dim r As PMDresult
            Dim ev As PMDEvent_internal
            ev.axis = EventStruct.axis
            ev.EventMask = EventStruct.EventMask
            r = PMDWaitForEvent(hDev, ev, timeout)
            CheckResult(r)
            EventStruct.axis = ev.axis
            EventStruct.EventMask = ev.EventMask
            Return r
        End Function

        Public Function GetDefault(ByVal code As PMDDefault) As UInt32
            Dim value As UInt32
            Dim datasize As UInt32
            datasize = 4
            CheckResult(PMDDeviceGetDefaultUInt32(hDev, code, value, datasize))
            Return value
        End Function

        Public Sub SetDefault(ByVal code As PMDDefault, ByVal value As UInt32)
            CheckResult(PMDDeviceSetDefaultUInt32(hDev, code, CType(value, UInt32), 4))
        End Sub

        'Task control methods
        Public ReadOnly Property TaskState() As PMDTaskState
            Get
                Dim state As PMDTaskState
                CheckResult(PMDTaskGetState(hDev, state))
                Return state
            End Get
        End Property

        Public Sub TaskStart()
            CheckResult(PMDTaskStart(hDev))
        End Sub

        Public Sub TaskStop()
            CheckResult(PMDTaskStop(hDev))
        End Sub
    End Class

    Public Class PMDAxis
        '*** Private data and utility functions ***
        Friend hAxis As IntPtr
        Friend status As PMDresult

        Friend Sub CheckResult(ByVal status As PMDresult)
            Me.status = status
            If (Not status = PMDresult.NOERROR) Then
                Dim e As New Exception("ERROR: PMDAxis " & PMDresultString(status))
                e.Data.Add("PMDresult", status)
                Throw e
            End If
        End Sub

        Friend Sub New()
            hAxis = PMDAxisAlloc()
            If (hAxis = 0) Then
                Throw New Exception("ERROR: PMD library: could not allocate axis object")
            End If
        End Sub


        '*** Public Methods ***
        Public Sub New(ByVal device As PMDDevice, ByVal AxisNumber As PMDAxisNumber)
            hAxis = PMDAxisAlloc()
            If (hAxis = 0) Then
                Throw New Exception("ERROR: PMD library: could not allocate axis object")
            End If
            CheckResult(PMDAxisOpen(hAxis, device.hDev, AxisNumber))
        End Sub

        Public Function AtlasAxis() As PMDAxis
            Dim Atlas As PMDAxis
            Atlas = New PMDAxis()
            Atlas.hAxis = PMDAxisAlloc()
            If (Atlas.hAxis = 0) Then
                Throw New Exception("ERROR: PMD library: could not allocate axis object")
            End If
            CheckResult(PMDAtlasAxisOpen(hAxis, Atlas.hAxis))
            Return Atlas
        End Function

        Protected Overrides Sub Finalize()
            If (Not hAxis = 0) Then
                PMDAxisClose(hAxis)
                PMDAxisFree(hAxis)
                hAxis = 0
            End If
        End Sub

        Public Sub Close()
            Me.Finalize()
        End Sub

        Public ReadOnly Property LastError() As PMDresult
            Get
                Return status
            End Get
        End Property

        Public Sub Reset()
            PMDReset(hAxis)
        End Sub

        ' Axis Class Properties and Methods
        Public Property Acceleration() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetAcceleration(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt32)
                CheckResult(PMDSetAcceleration(hAxis, value))
            End Set
        End Property

        Public ReadOnly Property ActiveMotorCommand() As Int16
            Get
                Dim tmp As Int16
                CheckResult(PMDGetActiveMotorCommand(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property ActiveOperatingMode() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetActiveOperatingMode(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property ActivityStatus() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetActivityStatus(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Property ActualPosition() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetActualPosition(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As Int32)
                CheckResult(PMDSetActualPosition(hAxis, value))
            End Set
        End Property

        Public Sub AdjustActualPosition(ByVal increment As Int32)
            CheckResult(PMDAdjustActualPosition(hAxis, increment))
        End Sub

        Public Property ActualPositionUnits() As PMDActualPositionUnits
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetActualPositionUnits(hAxis, tmp))
                Return CType(tmp, PMDActualPositionUnits)
            End Get
            Set(ByVal value As PMDActualPositionUnits)
                CheckResult(PMDSetActualPositionUnits(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public ReadOnly Property ActualVelocity() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetActualVelocity(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Function ReadAnalog(ByVal portID As Int16) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDReadAnalog(hAxis, portID, tmp))
            Return tmp
        End Function

        Public Sub GetAnalogCalibration(ByVal channel As PMDAnalogCalibration, _
                                        ByRef value As Int16)
            Dim tmp As Int16
            CheckResult(PMDGetAnalogCalibration(hAxis, channel, tmp))
            value = tmp
        End Sub

        Public Sub SetAnalogCalibration(ByVal channel As PMDAnalogCalibration, _
                                        ByVal value As Int16)
            CheckResult(PMDSetAnalogCalibration(hAxis, channel, value))
        End Sub

        Public Sub GetAuxiliaryEncoderSource(ByRef mode As PMDAuxiliaryEncoderMode, _
                                             ByRef AuxAxis As PMDAxisNumber)
            Dim tmp1 As Byte
            Dim tmp2 As UInt16
            CheckResult(PMDGetAuxiliaryEncoderSource(hAxis, tmp1, tmp2))
            mode = CType(tmp1, PMDAuxiliaryEncoderMode)
            AuxAxis = CType(tmp2, PMDAxisNumber)
        End Sub

        Public Sub SetAuxiliaryEncoderSource(ByVal mode As PMDAuxiliaryEncoderMode, _
                                             ByVal AuxAxis As PMDAxisNumber)
            CheckResult(PMDSetAuxiliaryEncoderSource(hAxis, CType(mode, Byte), CType(AuxAxis, UInt16)))
        End Sub


        Public Sub GetAxisOutMask(ByRef sourceAxis As PMDAxisNumber, _
                                  ByRef sourceRegister As PMDAxisOutRegister, _
                                  ByRef selectionMask As UInt16, _
                                  ByRef senseMask As UInt16)
            Dim tmp1 As UInt16
            Dim tmp2 As Byte
            Dim tmp3 As UInt16
            Dim tmp4 As UInt16
            CheckResult(PMDGetAxisOutMask(hAxis, tmp1, tmp2, tmp3, tmp4))
            sourceAxis = CType(tmp1, PMDAxisNumber)
            sourceRegister = CType(tmp2, PMDAxisOutRegister)
            selectionMask = tmp3
            senseMask = tmp4
        End Sub

        Public Sub SetAxisOutMask(ByVal sourceAxis As PMDAxisNumber, _
                                  ByVal sourceRegister As PMDAxisOutRegister, _
                                  ByVal selectionMask As UInt16, _
                                  ByVal senseMask As UInt16)
            CheckResult(PMDSetAxisOutMask(hAxis, CType(sourceAxis, UInt16), CType(sourceRegister, Byte), _
                                                  selectionMask, senseMask))
        End Sub


        Public Sub GetBreakpoint(ByRef BreakpointId As Int16, _
                                 ByRef SourceAxis As PMDAxisNumber, _
                                 ByRef Action As PMDBreakpointAction, _
                                 ByRef Trigger As PMDBreakpointTrigger)
            Dim tmp2 As UInt16
            Dim tmp3 As Byte
            Dim tmp4 As Byte
            CheckResult(PMDGetBreakpoint(hAxis, BreakpointId, tmp2, tmp3, tmp4))
            SourceAxis = CType(tmp2, PMDAxisNumber)
            Action = CType(tmp3, PMDBreakpointAction)
            Trigger = CType(tmp4, PMDBreakpointTrigger)
        End Sub

        Public Sub SetBreakpoint(ByVal BreakpointId As Int16, _
                                 ByVal SourceAxis As PMDAxisNumber, _
                                 ByVal Action As PMDBreakpointAction, _
                                 ByVal Trigger As PMDBreakpointTrigger)
            CheckResult(PMDSetBreakpoint(hAxis, BreakpointId, CType(SourceAxis, UInt16), _
                                          CType(Action, Byte), CType(Trigger, Byte)))
        End Sub

        Public Sub GetBreakpointUpdateMask(ByVal BreakpointId As Int16, _
                                           ByRef Mask As Int16)
            CheckResult(PMDGetBreakpointUpdateMask(hAxis, BreakpointId, Mask))
        End Sub

        Public Sub SetBreakpointUpdateMask(ByVal BreakpointId As Int16, _
                                           ByVal Mask As Int16)
            CheckResult(PMDSetBreakpointUpdateMask(hAxis, BreakpointId, Mask))
        End Sub

        Public Function GetBreakpointValue(ByVal BreakpointId As Int16) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetBreakpointValue(hAxis, BreakpointId, tmp))
            Return tmp
        End Function

        Public Sub SetBreakpointValue(ByVal BreakpointId As Int16, ByVal value As Int32)
            CheckResult(PMDSetBreakpointValue(hAxis, BreakpointId, value))
        End Sub

        Public Function ReadBuffer(ByVal BufferId As Int16) As Int32
            Dim tmp As Int32
            CheckResult(PMDReadBuffer(hAxis, BufferId, tmp))
            Return tmp
        End Function

        Public Function ReadBuffer16(ByVal BufferId As Int16) As Int16
            Dim tmp As Int16
            CheckResult(PMDReadBuffer(hAxis, BufferId, tmp))
            Return tmp
        End Function

        Public Sub WriteBuffer(ByVal BufferId As Int16, _
                                ByVal value As Int32)
            CheckResult(PMDWriteBuffer(hAxis, BufferId, value))
        End Sub

        Public Function GetBufferLength(ByVal BufferId As Int16) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetBufferLength(hAxis, BufferId, tmp))
            Return tmp
        End Function

        Public Sub SetBufferLength(ByVal BufferId As Int16, ByVal value As Int32)
            CheckResult(PMDSetBufferLength(hAxis, BufferId, value))
        End Sub

        Public Function GetBufferReadIndex(ByVal BufferId As Int16) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetBufferReadIndex(hAxis, BufferId, tmp))
            Return tmp
        End Function

        Public Sub SetBufferReadIndex(ByVal BufferId As Int16, ByVal value As Int32)
            CheckResult(PMDSetBufferReadIndex(hAxis, BufferId, value))
        End Sub

        Public Function GetBufferStart(ByVal BufferId As Int16) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetBufferStart(hAxis, BufferId, tmp))
            Return tmp
        End Function

        Public Sub SetBufferStart(ByVal BufferId As Int16, ByVal value As Int32)
            CheckResult(PMDSetBufferStart(hAxis, BufferId, value))
        End Sub

        Public Function GetBufferWriteIndex(ByVal BufferId As Int16) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetBufferWriteIndex(hAxis, BufferId, tmp))
            Return tmp
        End Function

        Public Sub SetBufferWriteIndex(ByVal BufferId As Int16, ByVal value As Int32)
            CheckResult(PMDSetBufferWriteIndex(hAxis, BufferId, value))
        End Sub

        Public ReadOnly Property BusVoltage() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetBusVoltage(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Function GetBusVoltageLimits(ByVal parameter As Int16) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetBusVoltageLimits(hAxis, parameter, tmp))
            Return tmp
        End Function

        Public Sub SetBusVoltageLimits(ByVal parameter As Int16, ByVal value As UInt16)
            CheckResult(PMDSetBusVoltageLimits(hAxis, parameter, value))
        End Sub

        Public Sub CalibrateAnalog(ByVal parameter As Int16)
            CheckResult(PMDCalibrateAnalog(hAxis, parameter))
        End Sub

        Public Sub GetCANMode(ByRef NodeId As Byte, ByRef TransmissionRate As PMDCANBaud)
            Dim tmp2 As Byte
            CheckResult(PMDGetCANMode(hAxis, NodeId, tmp2))
            TransmissionRate = CType(tmp2, PMDCANBaud)
        End Sub

        Public Sub SetCANMode(ByVal NodeId As Byte, ByVal TransmissionRate As PMDCANBaud)
            CheckResult(PMDSetCANMode(hAxis, NodeId, CType(TransmissionRate, Byte)))
        End Sub

        Public Property CaptureSource() As PMDCaptureSource
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetCaptureSource(hAxis, tmp))
                Return CType(tmp, PMDCaptureSource)
            End Get
            Set(ByVal value As PMDCaptureSource)
                CheckResult(PMDSetCaptureSource(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public ReadOnly Property CaptureValue() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetCaptureValue(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property Checksum() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetChecksum(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property CommandedAcceleration() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetCommandedAcceleration(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property CommandedPosition() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetCommandedPosition(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property CommandedVelocity() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetCommandedVelocity(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Property CommutationMode() As PMDCommutationMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetCommutationMode(hAxis, tmp))
                Return CType(tmp, PMDCommutationMode)
            End Get
            Set(ByVal value As PMDCommutationMode)
                CheckResult(PMDSetCommutationMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Function GetCurrent(ByVal parameter As PMDCurrent) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetCurrent(hAxis, CType(parameter, UInt16), tmp))
            Return tmp
        End Function

        Public Sub SetCurrent(ByVal parameter As PMDCurrent, ByVal value As UInt16)
            CheckResult(PMDSetCurrent(hAxis, CType(parameter, UInt16), value))
        End Sub

        Public Property CurrentControlMode() As PMDCurrentControlMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetCurrentControlMode(hAxis, tmp))
                Return CType(tmp, PMDCurrentControlMode)
            End Get
            Set(ByVal value As PMDCurrentControlMode)
                CheckResult(PMDSetCurrentControlMode(hAxis, CType(value, PMDCurrentControlMode)))
            End Set
        End Property

        Public Function GetCurrentFoldback(ByVal parameter As Int16) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetCurrentFoldback(hAxis, parameter, tmp))
            Return tmp
        End Function

        Public Sub SetCurrentFoldback(ByVal parameter As Int16, ByVal value As UInt16)
            CheckResult(PMDSetCurrentFoldback(hAxis, parameter, value))
        End Sub

        Public Function GetCurrentLoop(ByVal phase As PMDCurrentLoopNumber, _
                                    ByVal parameter As PMDCurrentLoopParameter) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetCurrentLoop(hAxis, CType(phase, Byte), CType(parameter, Byte), tmp))
            Return tmp
        End Function

        Public Sub SetCurrentLoop(ByVal phase As PMDCurrentLoopNumber, _
                                  ByVal parameter As PMDCurrentLoopParameter, _
                                  ByVal value As UInt16)
            CheckResult(PMDSetCurrentLoop(hAxis, CType(phase, Byte), CType(parameter, Byte), value))
        End Sub

        Public Function GetCurrentLoopValue(ByVal phase As PMDCurrentLoopNumber, _
                                            ByVal node As PMDCurrentLoopValueNode) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetCurrentLoopValue(hAxis, CType(phase, Byte), CType(node, Byte), tmp))
            Return tmp
        End Function

        Public Property Deceleration() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetDeceleration(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt32)
                CheckResult(PMDSetDeceleration(hAxis, value))
            End Set
        End Property

        ' Note that these are ION defaults, not Prodigy/CME defaults
        ' This should use an enum for the variable argument.
        Public Function GetMPDefault(ByVal variable As Int16) As UInt32
            Dim tmp As UInt32
            CheckResult(PMDGetDefault(hAxis, variable, tmp))
            Return tmp
        End Function

        Public Sub SetMPDefault(ByVal variable As Int16, ByVal value As UInt32)
            CheckResult(PMDSetDefault(hAxis, variable, value))
        End Sub

        Public ReadOnly Property DriveFaultStatus() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetDriveFaultStatus(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Sub ClearDriveFaultStatus()
            CheckResult(PMDClearDriveFaultStatus(hAxis))
        End Sub

        Public Sub GetDriveCommandMode(ByRef transport As Byte, ByRef format As Byte)
            CheckResult(PMDGetDriveCommandMode(hAxis, transport, format))
        End Sub

        Public Sub SetDriveCommandMode(ByVal transport As Byte, ByVal format As Byte)
            CheckResult(PMDSetDriveCommandMode(hAxis, transport, format))
        End Sub

        Public Function GetDriveFaultParameter(ByVal parameter As PMDDriveFaultParameter) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetDriveFaultParameter(hAxis, CType(parameter, UInt16), tmp))
            Return tmp
        End Function

        Public Function GetDriveValue(ByVal node As PMDDriveValue) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetDriveValue(hAxis, CType(node, UInt16), tmp))
            Return tmp
        End Function

        Public Sub SetDriveFaultParameter(ByVal parameter As PMDDriveFaultParameter, ByVal value As UInt16)
            CheckResult(PMDSetDriveFaultParameter(hAxis, CType(parameter, UInt16), value))
        End Sub

        Public Sub DriveNVRAM(ByVal parameter As PMDNVRAM, ByVal value As UInt16)
            CheckResult(PMDDriveNVRAM(hAxis, CType(parameter, UInt16), value))
        End Sub

        Public Function GetDrivePWM(ByVal parameter As PMDDrivePWM) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetDrivePWM(hAxis, CType(parameter, UInt16), tmp))
            Return tmp
        End Function

        Public Sub SetDrivePWM(ByVal parameter As PMDDrivePWM, ByVal value As UInt16)
            CheckResult(PMDSetDrivePWM(hAxis, CType(parameter, UInt16), value))
        End Sub

        Public Sub ClearInterrupt()
            CheckResult(PMDClearInterrupt(hAxis))
        End Sub

        Public Sub ClearPositionError()
            CheckResult(PMDClearPositionError(hAxis))
        End Sub

        Public ReadOnly Property DriveStatus() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetDriveStatus(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Property EncoderModulus() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetEncoderModulus(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetEncoderModulus(hAxis, value))
            End Set
        End Property

        Public Property EncoderSource() As PMDEncoderSource
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetEncoderSource(hAxis, tmp))
                Return CType(tmp, PMDEncoderSource)
            End Get
            Set(ByVal value As PMDEncoderSource)
                CheckResult(PMDSetEncoderSource(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Sub GetEncoderToStepRatio(ByRef counts As UInt16, ByRef steps As UInt16)
            CheckResult(PMDGetEncoderToStepRatio(hAxis, counts, steps))
        End Sub

        Public Sub SetEncoderToStepRatio(ByVal counts As UInt16, ByVal steps As UInt16)
            CheckResult(PMDSetEncoderToStepRatio(hAxis, counts, steps))
        End Sub

        Public Function GetEventAction(ByVal ActionEvent As PMDEventActionEvent) As PMDEventAction
            Dim tmp As UInt16
            CheckResult(PMDGetEventAction(hAxis, CType(ActionEvent, UInt16), tmp))
            Return CType(tmp, PMDEventAction)
        End Function

        Public Sub SetEventAction(ByVal ActionEvent As PMDEventActionEvent, ByVal Action As PMDEventAction)
            CheckResult(PMDSetEventAction(hAxis, CType(ActionEvent, UInt16), CType(Action, UInt16)))
        End Sub

        Public ReadOnly Property EventStatus() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetEventStatus(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Sub ResetEventStatus(ByVal StatusMask As UInt16)
            CheckResult(PMDResetEventStatus(hAxis, StatusMask))
        End Sub

        Public Function GetFeedbackParameter(ByVal parameter As PMDFeedbackParameter) As UInt32
            Dim tmp As UInt32
            CheckResult(PMDGetFeedbackParameter(hAxis, CType(parameter, UInt16), tmp))
            Return tmp
        End Function

        Public Sub SetFeedbackParameter(ByVal parameter As PMDFeedbackParameter, ByVal value As UInt32)
            CheckResult(PMDSetFeedbackParameter(hAxis, CType(parameter, UInt16), value))
        End Sub

        Public Function GetFOC(ByVal ControlLoop As PMDFOC_LoopNumber, _
                               ByVal parameter As PMDFOCLoopParameter) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetFOC(hAxis, CType(ControlLoop, Byte), CType(parameter, Byte), tmp))
            Return tmp
        End Function

        Public Sub SetFOC(ByVal ControlLoop As PMDFOC_LoopNumber, _
                          ByVal parameter As PMDFOCLoopParameter, _
                          ByVal value As UInt16)
            CheckResult(PMDSetFOC(hAxis, CType(ControlLoop, Byte), CType(parameter, Byte), value))
        End Sub

        Public Function GetFOCValue(ByVal ControlLoop As PMDFOC_LoopNumber, _
                                 ByVal node As PMDFOCValueNode) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetFOCValue(hAxis, CType(ControlLoop, Byte), CType(node, Byte), tmp))
            Return tmp
        End Function

        Public Property FaultOutMask() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetFaultOutMask(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetFaultOutMask(hAxis, value))
            End Set
        End Property

        Public Sub GetGearMaster(ByRef MasterAxis As PMDAxisNumber, ByRef source As PMDGearMasterSource)
            Dim tmp1 As UInt16
            Dim tmp2 As UInt16
            CheckResult(PMDGetGearMaster(hAxis, tmp1, tmp2))
            MasterAxis = CType(tmp1, PMDAxisNumber)
            source = CType(tmp2, PMDGearMasterSource)
        End Sub

        Public Sub SetGearMaster(ByVal MasterAxis As PMDAxisNumber, ByVal source As PMDGearMasterSource)
            CheckResult(PMDSetGearMaster(hAxis, CType(MasterAxis, UInt16), CType(source, UInt16)))
        End Sub

        Public Property GearRatio() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetGearRatio(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As Int32)
                CheckResult(PMDSetGearRatio(hAxis, value))
            End Set
        End Property

        Public Function GetHoldingCurrent(ByVal parameter As PMDHoldingCurrent) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDGetHoldingCurrent(hAxis, CType(parameter, UInt16), tmp))
            Return tmp
        End Function

        Public Sub SetHoldingCurrent(ByVal parameter As PMDHoldingCurrent, ByVal value As UInt16)
            CheckResult(PMDSetHoldingCurrent(hAxis, CType(parameter, UInt16), value))
        End Sub

        Public Sub InitializePhase()
            CheckResult(PMDInitializePhase(hAxis))
        End Sub

        Public ReadOnly Property InstructionError() As PMDInstructionError
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetInstructionError(hAxis, tmp))
                Return CType(tmp, PMDInstructionError)
            End Get
        End Property

        Public ReadOnly Property InterruptAxis() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetInterruptAxis(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Property InterruptMask() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetInterruptMask(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetInterruptMask(hAxis, value))
            End Set
        End Property

        Public Function ReadIO(ByVal address As UInt16) As UInt16
            Dim tmp As UInt16
            CheckResult(PMDReadIO(hAxis, address, tmp))
            Return tmp
        End Function

        Public Sub WriteIO(ByVal address As UInt16, ByVal value As UInt16)
            CheckResult(PMDWriteIO(hAxis, address, value))
        End Sub

        Public Property Jerk() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetJerk(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt32)
                CheckResult(PMDSetJerk(hAxis, value))
            End Set
        End Property

        Public Property MotionCompleteMode() As PMDMotionCompleteMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetMotionCompleteMode(hAxis, tmp))
                Return CType(tmp, PMDMotionCompleteMode)
            End Get
            Set(ByVal value As PMDMotionCompleteMode)
                CheckResult(PMDSetMotionCompleteMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Property MotorBias() As Int16
            Get
                Dim tmp As Int16
                CheckResult(PMDGetMotorBias(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As Int16)
                CheckResult(PMDSetMotorBias(hAxis, value))
            End Set
        End Property

        Public Property MotorCommand() As Int16
            Get
                Dim tmp As Int16
                CheckResult(PMDGetMotorCommand(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As Int16)
                CheckResult(PMDSetMotorCommand(hAxis, value))
            End Set
        End Property

        Public Property MotorLimit() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetMotorLimit(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetMotorLimit(hAxis, value))
            End Set
        End Property

        Public Property MotorType() As PMDMotorType
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetMotorType(hAxis, tmp))
                Return CType(tmp, PMDMotorType)
            End Get
            Set(ByVal value As PMDMotorType)
                CheckResult(PMDSetMotorType(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Sub MultiUpdate(ByVal AxisMask As UInt16)
            CheckResult(PMDMultiUpdate(hAxis, AxisMask))
        End Sub

        Public Sub NoOperation()
            CheckResult(PMDNoOperation(hAxis))
        End Sub

        Public Property OperatingMode() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetOperatingMode(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetOperatingMode(hAxis, value))
            End Set
        End Property

        Public Property OutputMode() As PMDMotorOutputMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetOutputMode(hAxis, tmp))
                Return CType(tmp, PMDMotorOutputMode)
            End Get
            Set(ByVal value As PMDMotorOutputMode)
                CheckResult(PMDSetOutputMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Property OvertemperatureLimit() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetOvertemperatureLimit(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetOvertemperatureLimit(hAxis, value))
            End Set
        End Property

        Public Property PWMFrequency() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPWMFrequency(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetPWMFrequency(hAxis, value))
            End Set
        End Property

        Public Property PhaseAngle() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPhaseAngle(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetPhaseAngle(hAxis, value))
            End Set
        End Property

        Public Function GetPhaseCommand(ByVal phase As PMDPhaseCommand) As Int16
            Dim tmp As Int16
            CheckResult(PMDGetPhaseCommand(hAxis, CType(phase, UInt16), tmp))
            Return tmp
        End Function

        Public Property PhaseCorrectionMode() As PMDPhaseCorrectionMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPhaseCorrectionMode(hAxis, tmp))
                Return CType(tmp, PMDPhaseCorrectionMode)
            End Get
            Set(ByVal value As PMDPhaseCorrectionMode)
                CheckResult(PMDSetPhaseCorrectionMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Property PhaseCounts() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPhaseCounts(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetPhaseCounts(hAxis, value))
            End Set
        End Property

        Public Property PhaseInitializeMode() As PMDPhaseInitializeMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPhaseInitializeMode(hAxis, tmp))
                Return CType(tmp, PMDPhaseInitializeMode)
            End Get
            Set(ByVal value As PMDPhaseInitializeMode)
                CheckResult(PMDSetPhaseInitializeMode(hAxis, value))
            End Set
        End Property

        Public Property PhaseInitializeTime() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPhaseInitializeTime(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetPhaseInitializeTime(hAxis, value))
            End Set
        End Property

        Public Property PhaseOffset() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPhaseOffset(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetPhaseOffset(hAxis, value))
            End Set
        End Property

        Public Property PhasePrescale() As PMDPhasePrescaleMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetPhasePrescale(hAxis, tmp))
                Return CType(tmp, PMDPhasePrescaleMode)
            End Get
            Set(ByVal value As PMDPhasePrescaleMode)
                CheckResult(PMDSetPhasePrescale(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Property Position() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetPosition(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As Int32)
                CheckResult(PMDSetPosition(hAxis, value))
            End Set
        End Property

        Public ReadOnly Property PositionError() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetPositionError(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Property PositionErrorLimit() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetPositionErrorLimit(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt32)
                CheckResult(PMDSetPositionErrorLimit(hAxis, value))
            End Set
        End Property

        Public Function GetPositionLoop(ByVal parameter As PMDPositionLoop) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetPositionLoop(hAxis, CType(parameter, UInt16), tmp))
            Return tmp
        End Function

        Public Sub SetPositionLoop(ByVal parameter As PMDPositionLoop, ByVal value As Int32)
            CheckResult(PMDSetPositionLoop(hAxis, CType(parameter, UInt16), value))
        End Sub

        Public Function GetPositionLoopValue(ByVal parameter As PMDPositionLoopValueNode) As Int32
            Dim tmp As Int32
            CheckResult(PMDGetPositionLoopValue(hAxis, CType(parameter, UInt16), tmp))
            Return tmp
        End Function

        Public Property ProfileMode() As PMDProfileMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetProfileMode(hAxis, tmp))
                Return CType(tmp, PMDProfileMode)
            End Get
            Set(ByVal value As PMDProfileMode)
                CheckResult(PMDSetProfileMode(hAxis, CType(value, PMDProfileMode)))
            End Set
        End Property

        Public Sub RestoreOperatingMode()
            CheckResult(PMDRestoreOperatingMode(hAxis))
        End Sub

        Public Property SPIMode() As PMDSPIMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetSPIMode(hAxis, tmp))
                Return CType(tmp, PMDSPIMode)
            End Get
            Set(ByVal value As PMDSPIMode)
                CheckResult(PMDSetSPIMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Property SampleTime() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetSampleTime(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt32)
                CheckResult(PMDSetSampleTime(hAxis, value))
            End Set
        End Property

        Public Sub GetSerialPortMode(ByRef baud As PMDSerialBaud, _
                                     ByRef parity As PMDSerialParity, _
                                     ByRef StopBits As PMDSerialStopBits, _
                                     ByRef protocol As PMDSerialProtocol, _
                                     ByRef MultiDropId As Byte)
            Dim tmp1 As Byte
            Dim tmp2 As Byte
            Dim tmp3 As Byte
            Dim tmp4 As Byte
            CheckResult(PMDGetSerialPortMode(hAxis, tmp1, tmp2, tmp3, tmp4, MultiDropId))
            baud = CType(tmp1, PMDSerialBaud)
            parity = CType(tmp2, PMDSerialParity)
            StopBits = CType(tmp3, PMDSerialStopBits)
            protocol = CType(tmp4, PMDSerialProtocol)
        End Sub

        Public Sub SetSerialPortMode(ByVal baud As PMDSerialBaud, _
                                     ByVal parity As PMDSerialParity, _
                                     ByVal StopBits As PMDSerialStopBits, _
                                     ByVal protocol As PMDSerialProtocol, _
                                     ByVal MultiDropId As Byte)
            CheckResult(PMDSetSerialPortMode(hAxis, CType(baud, Byte), CType(parity, Byte), _
                                              CType(StopBits, Byte), CType(protocol, Byte), _
                                              MultiDropId))
        End Sub

        Public Property SettleTime() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetSettleTime(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetSettleTime(hAxis, value))
            End Set
        End Property

        Public Property SettleWindow() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetSettleWindow(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetSettleWindow(hAxis, value))
            End Set
        End Property

        Public Property SignalSense() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetSignalSense(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetSignalSense(hAxis, value))
            End Set
        End Property

        Public ReadOnly Property SignalStatus() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetSignalStatus(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Property StartVelocity() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetStartVelocity(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt32)
                CheckResult(PMDSetStartVelocity(hAxis, value))
            End Set
        End Property

        ' See documentation for step ranges
        Public Property StepRange() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetStepRange(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetStepRange(hAxis, value))
            End Set
        End Property

        Public Property StopMode() As PMDStopMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetStopMode(hAxis, tmp))
                Return CType(tmp, PMDStopMode)
            End Get
            Set(ByVal value As PMDStopMode)
                CheckResult(PMDSetStopMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Property SynchronizationMode() As PMDSynchronizationMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetSynchronizationMode(hAxis, tmp))
                Return CType(tmp, PMDSynchronizationMode)
            End Get
            Set(ByVal value As PMDSynchronizationMode)
                CheckResult(PMDSetSynchronizationMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public ReadOnly Property Temperature() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetTemperature(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property Time() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetTime(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public ReadOnly Property TraceCount() As UInt32
            Get
                Dim tmp As UInt32
                CheckResult(PMDGetTraceCount(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Property TraceMode() As PMDTraceMode
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetTraceMode(hAxis, tmp))
                Return CType(tmp, PMDTraceMode)
            End Get
            Set(ByVal value As PMDTraceMode)
                CheckResult(PMDSetTraceMode(hAxis, CType(value, UInt16)))
            End Set
        End Property

        Public Property TracePeriod() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetTracePeriod(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetTracePeriod(hAxis, value))
            End Set
        End Property

        Public Sub GetTraceStart(ByRef triggerAxis As PMDAxisNumber, _
                                 ByRef condition As PMDTraceCondition, _
                                 ByRef bit As Byte, _
                                 ByRef state As PMDTraceTriggerState)
            Dim tmp1 As UInt16
            Dim tmp2 As Byte
            Dim tmp4 As Byte
            CheckResult(PMDGetTraceStart(hAxis, tmp1, tmp2, bit, tmp4))
            triggerAxis = CType(tmp1, PMDAxisNumber)
            condition = CType(tmp2, PMDTraceCondition)
            state = CType(tmp4, PMDTraceTriggerState)
        End Sub

        Public Sub SetTraceStart(ByVal triggerAxis As PMDAxisNumber, _
                                 ByVal condition As PMDTraceCondition, _
                                 ByVal bit As Byte, _
                                 ByVal state As PMDTraceTriggerState)
            CheckResult(PMDSetTraceStart(hAxis, CType(triggerAxis, UInt16), CType(condition, Byte), _
                                          bit, CType(state, Byte)))
        End Sub

        Public ReadOnly Property TraceStatus() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetTraceStatus(hAxis, tmp))
                Return tmp
            End Get
        End Property

        Public Sub GetTraceStop(ByRef triggerAxis As PMDAxisNumber, _
                                ByRef condition As PMDTraceCondition, _
                                ByRef bit As Byte, _
                                ByRef state As PMDTraceTriggerState)
            Dim tmp1 As UInt16
            Dim tmp2 As Byte
            Dim tmp4 As Byte
            CheckResult(PMDGetTraceStop(hAxis, tmp1, tmp2, bit, tmp4))
            triggerAxis = CType(tmp1, PMDAxisNumber)
            condition = CType(tmp2, PMDTraceCondition)
            state = CType(tmp4, PMDTraceTriggerState)
        End Sub

        Public Sub SetTraceStop(ByVal triggerAxis As PMDAxisNumber, _
                                ByVal condition As PMDTraceCondition, _
                                ByVal bit As Byte, _
                                ByVal state As PMDTraceTriggerState)
            CheckResult(PMDSetTraceStop(hAxis, CType(triggerAxis, UInt16), CType(condition, Byte), _
                                          bit, CType(state, Byte)))
        End Sub

        Public Sub GetTraceVariable(ByVal VariableNumber As PMDTraceVariableNumber, _
                                    ByRef TraceAxis As PMDAxisNumber, _
                                    ByRef variable As PMDTraceVariable)
            Dim tmp3 As UInt16
            Dim tmp4 As Byte
            CheckResult(PMDGetTraceVariable(hAxis, CType(VariableNumber, UInt16), tmp3, tmp4))
            TraceAxis = CType(tmp3, PMDAxisNumber)
            variable = CType(tmp4, PMDTraceVariable)
        End Sub

        Public Sub SetTraceVariable(ByVal VariableNumber As PMDTraceVariableNumber, _
                                    ByVal TraceAxis As PMDAxisNumber, _
                                    ByVal variable As PMDTraceVariable)
            CheckResult(PMDSetTraceVariable(hAxis, CType(VariableNumber, UInt16), _
                                            CType(TraceAxis, UInt16), CType(variable, Byte)))
        End Sub

        Public Property TrackingWindow() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetTrackingWindow(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetTrackingWindow(hAxis, value))
            End Set
        End Property

        Public Sub Update()
            CheckResult(PMDUpdate(hAxis))
        End Sub

        Public Property UpdateMask() As UInt16
            Get
                Dim tmp As UInt16
                CheckResult(PMDGetUpdateMask(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As UInt16)
                CheckResult(PMDSetUpdateMask(hAxis, value))
            End Set
        End Property

        Public Property Velocity() As Int32
            Get
                Dim tmp As Int32
                CheckResult(PMDGetVelocity(hAxis, tmp))
                Return tmp
            End Get
            Set(ByVal value As Int32)
                CheckResult(PMDSetVelocity(hAxis, value))
            End Set
        End Property

        Public Sub GetVersion(ByRef family As UInt16, _
                                ByRef MotorType As PMDMotorTypeVersion, _
                                ByRef NumberAxes As UInt16, _
                                ByRef special_and_chip_count As UInt16, _
                                ByRef custom As UInt16, _
                                ByRef major As UInt16, _
                                ByRef minor As UInt16)
            Dim mtype As UInt16
            CheckResult(PMDGetVersion(hAxis, family, mtype, NumberAxes, special_and_chip_count, _
                                       custom, major, minor))
            MotorType = CType(mtype, PMDMotorTypeVersion)
        End Sub

    End Class 'Axis

    Public Class PMDMemory
        '*** Private data and utility functions ***
        Friend hMem As IntPtr
        Friend DataSize As PMDDataSize
        Friend status As PMDresult

        Friend Sub CheckResult(ByVal status As PMDresult)
            Me.status = status
            If (Not status = PMDresult.NOERROR) Then
                Dim e As New Exception("ERROR: PMDMemory " & PMDresultString(status))
                e.Data.Add("PMDresult", status)
                Throw e
            End If
        End Sub

        '*** Public Methods ***
        Public Sub New(ByVal device As PMDDevice, _
                       ByVal DataSize As PMDDataSize, _
                       ByVal MemType As PMDMemoryType)
            hMem = PMDMemoryAlloc()
            If (hMem = 0) Then
                Throw New Exception("ERROR: PMD library: could not allocate memory object")
            End If
            Me.DataSize = DataSize
            CheckResult(PMDMemoryOpen(hMem, device.hDev, DataSize, MemType))
        End Sub

        Protected Overrides Sub Finalize()
            If (Not hMem = 0) Then
                PMDMemoryClose(hMem)
                PMDMemoryFree(hMem)
                hMem = 0
            End If
        End Sub

        Public Sub Close()
            Me.Finalize()
        End Sub

        Public ReadOnly Property LastError() As PMDresult
            Get
                Return status
            End Get
        End Property

        ' NB only 32 bit reads are supported at this time
        Public Sub Read(ByRef data() As UInt32, ByVal offset As UInt32, ByVal length As UInt32)
            CheckResult(PMDMemoryRead(hMem, data(0), offset, length))
        End Sub

        Public Sub Write(ByRef data() As UInt32, ByVal offset As UInt32, ByVal length As UInt32)
            CheckResult(PMDMemoryWrite(hMem, data(0), offset, length))
        End Sub


    End Class 'PMDMemory

End Module