#ifndef NTBrainBitBlack_h
#define NTBrainBitBlack_h

#include <Foundation/Foundation.h>
#include "NTBrainBitBlack.h"
#include "NTTypes.h"
#include "NTBrainBit.h"

@interface NTBrainBitBlack : NTBrainBit
- (instancetype)init NS_UNAVAILABLE;

@property (NS_NONATOMIC_IOSONLY) NTSensorAccelerometerSensitivity AccSens;      // mems, callibri, BB,Handbat etc
@property (NS_NONATOMIC_IOSONLY) NTSensorGyroscopeSensitivity GyroSens;         // ||..||..||
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyFPG;      // FPG
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyMEMS;     // мемс
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorAmpMode AmpMode;     // не у всех
@property (NS_NONATOMIC_IOSONLY) NTIrAmplitude  IrAmplitudeHeadband;        //hb
@property (NS_NONATOMIC_IOSONLY) NTRedAmplitude RedAmplitudeHeadband;       //hb


- (void) setMEMSDataCallback:(void (^_Nullable)(NSArray<NTMEMSData*>*_Nonnull))callback;            // mems
- (void) setAmpModeCallback:(void (^_Nullable)(NTSensorAmpMode))callback;       // ne u vseh
- (void) setFPGDataCallbackNeuroSmart:(void (^_Nullable)(NSArray<NTFPGData*>*_Nonnull))callback;

-(void) PingNeuroSmart:(Byte)marker;        // ns

@end

#endif /* NTBrainBitBlack_h */


