#ifndef PMD_MotorSetup_Magellan
#define	PMD_MotorSetup_Magellan

//  PMDMotorSetup.h -- Basic motor setup functions
//
//  Performance Motion Devices, Inc.
//

#if defined(__cplusplus)
extern "C" {
#endif

PMDresult PMDMotorInit(PMDAxisHandle* phAxis, PMDProductType product);

PMDresult PMDSetup_BrushlessDC(PMDAxisHandle* phAxis,PMDProductType product);
PMDresult PMDSetup_Microstepping(PMDAxisHandle* phAxis,PMDProductType product);
PMDresult PMDSetup_DCBrush(PMDAxisHandle* phAxis,PMDProductType product);
PMDresult PMDSetup_Step(PMDAxisHandle* phAxis);
void SetupPositionLoop(PMDAxisHandle* phAxis);
void SetupCurrentLoop(PMDAxisHandle* phAxis);

#if defined(__cplusplus)
}
#endif

#endif	// PMD_MotorSetup

