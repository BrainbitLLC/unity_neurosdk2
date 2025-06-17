#ifndef NTHeadphones2_h
#define NTHeadphones2_h

#import <Foundation/Foundation.h>
#include "NTSensor.h"

@interface NTHeadphones2: NTSensor
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyMEMS;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyFPG;
@property (NS_NONATOMIC_IOSONLY) NTSensorAccelerometerSensitivity AccSens;
@property (NS_NONATOMIC_IOSONLY) NTSensorGyroscopeSensitivity GyroSens;
@property (NS_NONATOMIC_IOSONLY) NTIrAmplitude IrAmplitude;
@property (NS_NONATOMIC_IOSONLY) NTRedAmplitude RedAmplitude;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorAmpMode AmpMode;
@property (NS_NONATOMIC_IOSONLY) NTHeadphones2AmplifierParam* _Nonnull AmplifierParam;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyResist;


- (void) setMEMSDataCallback:(void (^_Nullable)(NSArray<NTMEMSData*>*_Nonnull))callback;
- (void) setFPGDataCallback:(void (^_Nullable)(NSArray<NTFPGData*>*_Nonnull))callback;
- (void) setAmpModeCallback:(void (^_Nullable)(NTSensorAmpMode))callback;
- (void) setResistCallback:(void (^_Nullable)(NSArray<NTHeadphones2ResistData*>*_Nonnull))callback;
- (void) setSignalDataCallback:(void (^_Nullable)(NSArray<NTHeadphones2SignalData*>*_Nonnull))callback;


-(void) PingNeuroSmart:(Byte)marker;

@end

#endif /* NTHeadphones2_h */
