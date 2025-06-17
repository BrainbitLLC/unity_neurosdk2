
#ifndef NTHandband_h
#define NTHandband_h

#import <Foundation/Foundation.h>
#import "NTTypes.h"
#include "NTSensor.h"

@interface NTHeadband : NTSensor
 
- (instancetype _Nonnull )init NS_UNAVAILABLE;

@property (NS_NONATOMIC_IOSONLY) NTSensorAccelerometerSensitivity AccSens;
@property (NS_NONATOMIC_IOSONLY) NTSensorGyroscopeSensitivity GyroSens;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyFPG;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyMEMS;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorAmpMode AmpMode;
@property (NS_NONATOMIC_IOSONLY) NTIrAmplitude  IrAmplitudeFPGSensor;
@property (NS_NONATOMIC_IOSONLY) NTRedAmplitude RedAmplitudeFPGSensor;

-(void) PingNeuroSmart:(Byte)marker;

- (void) setMEMSDataCallback:(void (^_Nullable)(NSArray<NTMEMSData*>*_Nonnull))callback;
- (void) setAmpModeCallback:(void (^_Nullable)(NTSensorAmpMode))callback;
- (void) setFPGDataCallbackNeuroSmart:(void (^_Nullable)(NSArray<NTFPGData*>*_Nonnull))callback;

- (void) setResistCallbackHeadband:(void (^_Nullable)(NTHeadbandResistData*_Nonnull))callback;
- (void) setSignalCallbackHeadband:(void (^_Nullable)(NSArray<NTHeadbandSignalData*>*_Nonnull))callback;

@end


#endif /* NTHandband_h */
