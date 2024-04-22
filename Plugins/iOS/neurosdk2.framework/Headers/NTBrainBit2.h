//
//  NTBrainBit2.h
//  neurosdk
//
//  Created by Aseatari on 20.03.2024.
//

#ifndef NTBrainBit2_h
#define NTBrainBit2_h
#import <Foundation/Foundation.h>
#import "NTTypes.h"
#include "NTSensor.h"

@interface NTBrainBit2 : NTSensor
- (instancetype _Nonnull )init NS_UNAVAILABLE;

+ (NSNumber*_Nonnull) getMaxChCount;

@property (NS_NONATOMIC_IOSONLY) NTBrainBit2AmplifierParam* _Nonnull AmplifierParam;
@property (NS_NONATOMIC_IOSONLY, readonly) NSArray<NTEEGChannelInfo *>* _Nonnull SupportedChannels;

@property (NS_NONATOMIC_IOSONLY) NTSensorAccelerometerSensitivity AccSens;
@property (NS_NONATOMIC_IOSONLY) NTSensorGyroscopeSensitivity GyroSens;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyFPG;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyMEMS;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyResist;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorAmpMode AmpMode;
@property (NS_NONATOMIC_IOSONLY) NTIrAmplitude  IrAmplitude;
@property (NS_NONATOMIC_IOSONLY) NTRedAmplitude RedAmplitude;


- (void) setMEMSDataCallback:(void (^_Nullable)(NSArray<NTMEMSData*>*_Nonnull))callback;
- (void) setAmpModeCallback:(void (^_Nullable)(NTSensorAmpMode))callback;
- (void) setFPGDataCallback:(void (^_Nullable)(NSArray<NTFPGData*>*_Nonnull))callback;
- (void) setSignalDataCallback:(void (^_Nullable)(NSArray<NTSignalChannelsData*>*_Nonnull))callback;
- (void) setResistDataCallback:(void (^_Nullable)(NSArray<NTResistRefChannelsData*>*_Nonnull))callback;

-(void) PingNeuroSmart:(Byte)marker;

@end

#endif /* NTBrainBit2_h */
