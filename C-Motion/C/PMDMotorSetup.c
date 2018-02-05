// ****************************************************************
// PMDMotorSetup_Magellan.c : Defines standard motor setup routines
//
// Performance Motion Devices, Inc.
//

#include "c-motion.h"
#include "PMDMotorSetup.h"
#include "PMDMB.h"

#define ALGO_PHASE_INIT  // Brushless motors only


//*****************************************************************************
PMDresult PMDMotorInit(PMDAxisHandle* phAxis, PMDProductType product)
{
    PMDresult result;
    PMDuint16 motor_type;

    result = PMDGetMotorType(phAxis, &motor_type);

    if (result!=PMD_NOERROR)
        return result;

    switch (motor_type)
    {
        case PMDMotorTypeBrushlessDC3Phase:    
            result=PMDSetup_BrushlessDC(phAxis,product);   
            break;

        case PMDMotorTypeBrushlessDC2Phase:    
            result=PMDSetup_BrushlessDC(phAxis,product);   
            break;

        case PMDMotorTypeMicrostep3Phase:  
            result=PMDSetup_Microstepping(phAxis,product); 
            break;

        case PMDMotorTypeMicrostep2Phase:  
            result=PMDSetup_Microstepping(phAxis,product); 
            break;

        case PMDMotorTypeStep:                  
            result=PMDSetup_Step(phAxis);          
            break;

        case PMDMotorTypeDCBrush:               
            result=PMDSetup_DCBrush(phAxis,product);       
            break;
    }

    return result;
}

//*****************************************************************************
PMDresult PMDSetup_BrushlessDC(PMDAxisHandle* phAxis, PMDProductType product)
{
    PMDuint16 status;
    PMDuint16 amask = 1<<(phAxis->axis);
    PMDresult result;
    
    if ((product==PMDProductTypeProdigy)||(product==PMDProductTypeProdigyCME))        
    { 
        PMDMBSetDACOutputEnable(phAxis,PMDEnable);
        PMDMBSetAmplifierEnable(phAxis, amask, amask);
    }   
    
    // uncomment this line if DAC output is to be used on this axis
    PMDSetOutputMode(phAxis,PMDMotorOutputBipolarDAC);
    
    
    // Enable AxisEnable and MotorOuput only for now
    result = PMDSetOperatingMode(phAxis,PMDOperatingModeAxisEnabledMask|PMDOperatingModeMotorOutputEnabledMask); 
    if (result!=PMD_ERR_OK) 
        return result;
    // ---------------------------------------------------
    // Setup and initialize commutation
    // ---------------------------------------------------
    // TO DO:   Use appropriate values below
    PMDSetPhaseCounts( phAxis, 1000 );
    PMDSetPhaseCorrectionMode( phAxis, PMDDisable );

    // TO DO:   if encoder or hall sensor inputs need to be inverted use
    //          the SetSignalSense command to achieve this
    PMDSetSignalSense( phAxis, 0x0001 );

    //TO DO:   Selection Hall-based or Sinusoidal Commutation
    //PMDSetCommutationMode( phAxis, PMDCommutationMode_HallBased );
    PMDSetCommutationMode( phAxis, PMDCommutationModeSinusoidal );
    
    PMDSetPhasePrescale( phAxis,PMDPhasePrescaleOff); 
    // PhasePresale is used when the value of "encoder_counts/electrical_cycle" (aka PhaseCounts) exceeds 32767.
      
#ifdef ALGO_PHASE_INIT
    PMDSetPhaseInitializeMode( phAxis, PMDPhaseInitializeModeAlgorithmic );
    
    // TO DO:   Use appropriate values below
    PMDSetPhaseInitializeTime(phAxis, 6000);
    PMDSetMotorCommand(phAxis, 3000);
    PMDInitializePhase( phAxis );
    status=0;
    while(!status)   // poll ActivityStatus to determine when Phase Initilization is complete
    {
        PMDGetActivityStatus(phAxis, &status);
        status&=PMDActivityPhasingInitializedMask;
    }   
//  PMDprintf("Algorithmic Phase Initialization Completed\n");
#else
    PMDSetPhaseInitializeMode( phAxis, PMDPhaseInitializeMode_HallBased );
    PMDInitializePhase(phAxis);
#endif  
    
    SetupPositionLoop(phAxis);
    PMDClearPositionError(phAxis);
    PMDUpdate(phAxis);
    
    //  The ION Digital Drive has a current loop and the user has the choice of enabling it or not.
    if (product == PMDProductTypeION || product == PMDProductTypeIONCME)
    {
        SetupCurrentLoop(phAxis);
    }   
    
    result=PMDSetOperatingMode( phAxis, PMDOperatingModeAllEnabledMask );
    
    return result;
}

//*****************************************************************************
PMDresult PMDSetup_Microstepping(PMDAxisHandle* phAxis, PMDProductType product)
{
    PMDuint16 amask = 1<<(phAxis->axis);
    PMDresult result;
    
    if (product==PMDProductTypeProdigy)   //Prodigy board 
    { 
        PMDMBSetDACOutputEnable(phAxis,PMDEnable);
        PMDMBSetAmplifierEnable(phAxis, amask, amask);
    }   
    
    // uncomment this line if DAC output is to be used on this axis and it is not an ION
    //PMDSetOutputMode(phAxis,PMDMotorOutputBipolarDAC);
    
    // ---------------------------------------------------
    // Setup microstepping commutation
    // ---------------------------------------------------
    // TO DO:   Use appropriate values below
    PMDSetPhaseCounts( phAxis, 256 );  // When MicroStepping, PhaseCounts= microsteps/electrical cycle where 1 electrical cycle=4 whole steps.
    
    // uncomment these two lines if encoder is present
    //PMDSetEncoderSource(phAxis, PMDEncoderSourceIncremental);
    //PMDSetEncoderToStepRatio( phAxis, 2000, 12800 );

    // ---------------------------------------------------
    // Final required motor setup instructions
    // ---------------------------------------------------
    if (product == PMDProductTypeION || product == PMDProductTypeIONCME)
    {
        SetupCurrentLoop(phAxis);
    }   
    
    result=PMDSetOperatingMode( phAxis, PMDOperatingModeAllEnabledMask );

    if(result!=PMD_ERR_OK) 
        return result;

    // TO DO:   Use appropriate values below
    PMDSetMotorCommand( phAxis, 16000 );
    PMDSetCurrent( phAxis, PMDCurrentHoldingCurrent, 8000 );
    PMDSetCurrent( phAxis, PMDCurrentHoldingDelay, 10000 );
    
    result=PMDUpdate( phAxis);
    return result;
}

//*****************************************************************************
PMDresult PMDSetup_DCBrush(PMDAxisHandle* phAxis, PMDProductType product)
{
    PMDuint16 amask = 1<<(phAxis->axis);
    PMDresult result;
    
    if ((product==PMDProductTypeProdigy)||(product==PMDProductTypeProdigyCME))    //Prodigy board 
    { 
        PMDMBSetDACOutputEnable(phAxis,PMDEnable);
        PMDMBSetAmplifierEnable(phAxis, amask, amask);
    }   
    
    // uncomment this line if DAC output is to be used on this axis
    //PMDSetOutputMode(phAxis,PMDMotorOutputBipolarDAC);
    
    
    // Enable AxisEnable and MotorOuput only for now
    //PMDSetOperatingMode(phAxis,PMDOperatingModeAxisEnabledMask|PMDOperatingModeMotorOutputEnabledMask); 

    // TO DO:   if encoder needs to be inverted use
    //          the SetSignalSense command to achieve this
    //PMDSetSignalSense( phAxis, 0x0001 );

    SetupPositionLoop(phAxis);
    PMDClearPositionError(phAxis);
    PMDUpdate(phAxis);
    
    if (product == PMDProductTypeION || product == PMDProductTypeIONCME)
    {
        SetupCurrentLoop(phAxis);
    }   
    
    result=PMDSetOperatingMode( phAxis, PMDOperatingModeAllEnabledMask );
    
    return result;
    
}

//*****************************************************************************
PMDresult PMDSetup_Step(PMDAxisHandle* phAxis)
{
    PMDresult result;
    
    // TO DO: uncomment these two lines if encoder is present
    //PMDSetEncoderSource(phAxis, PMDEncoderSourceIncremental);
    //PMDSetEncoderToStepRatio( phAxis, 2000, 12800 );
    
    // TO DO:   Use appropriate value below
    //PMDSetStepRange( phAxis, 4 );
    
    PMDSetCurrent( phAxis, PMDCurrentHoldingCurrent, 8000 );  // This can be use to delay the AtReset signal
        
    result=PMDSetOperatingMode( phAxis, PMDOperatingModeAllEnabledMask );
    return result;
}

//*********************************************************************
void SetupPositionLoop(PMDAxisHandle* phAxis)
{
    // TO DO:   Use appropriate value below
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopProportionalGain, 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopIntegratorGain,  0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopIntegratorLimit, 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopDerivativeGain, 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopDerivativeTime, 1);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopOutputGain, 65535);

    
    
    // TO DO:   Use appropriate value below, these settings may not be needed
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopVelocityFeedforwardGain, 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopAccelerationFeedforwardGain, 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad1EnableFilter , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad1B0 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad1B1 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad1B2 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad1A1 , 0); 
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad1A2 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad1K , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad2EnableFilter , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad2B0 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad2B1 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad2B2 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad2A1 , 0); 
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad2A2 , 0);
    PMDSetPositionLoop         (phAxis,  PMDPositionLoopBiquad2K , 0);
    //PMDClearPositionError(phAxis);
    PMDUpdate(phAxis);
}   

//******************************************************************************
void SetupCurrentLoop(PMDAxisHandle* phAxis)
{
    
    // ---------------------------------------------------
    // Setup and select current loop or FOC
    // ---------------------------------------------------
    // TO DO:   Update with the appropriate parameter values
    //          before executing this code
    PMDSetCurrentLoop( phAxis, PMDCurrentLoopBoth, PMDCurrentLoopProportionalGain, 902 );
    PMDSetCurrentLoop( phAxis, PMDCurrentLoopBoth, PMDCurrentLoopIntegralGain, 408 );
    PMDSetCurrentLoop( phAxis, PMDCurrentLoopBoth, PMDCurrentLoopIntegralSumLimit, 68 );

    PMDSetFOC( phAxis, PMDFOCBoth, PMDFOCProportionalGain, 0 );
    PMDSetFOC( phAxis, PMDFOCBoth, PMDFOCIntegralGain, 0 );
    PMDSetFOC( phAxis, PMDFOCBoth, PMDFOCIntegralSumLimit, 0 );

    // TO DO:   Select desired mode
    PMDSetCurrentControlMode( phAxis, PMDCurrentControlModeCurrentLoop );
    //PMDSetCurrentControlMode( phAxis, PMDCurrentControlModeFOC );
    PMDUpdate( phAxis );
}   
