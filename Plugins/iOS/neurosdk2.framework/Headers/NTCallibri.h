#ifndef NTCallibri_h
#define NTCallibri_h

#include "NTSensor.h"

@interface NTCallibri : NTSensor

@property (NS_NONATOMIC_IOSONLY, readonly) NSArray<NSNumber *>*_Nullable  SupportedFilters;
@property (NS_NONATOMIC_IOSONLY, copy) NSArray<NSNumber *>*_Nullable  HardwareFilters;
@property (NS_NONATOMIC_IOSONLY) NTSensorExternalSwitchInput ExtSwInput;
@property (NS_NONATOMIC_IOSONLY) NTSensorADCInput ADCInput;
@property (NS_NONATOMIC_IOSONLY) NTSensorAccelerometerSensitivity AccSens;
@property (NS_NONATOMIC_IOSONLY) NTSensorGyroscopeSensitivity GyroSens;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyMEMS;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyEnvelope;
@property (NS_NONATOMIC_IOSONLY, readonly) NTCallibriStimulatorMAState* _Nullable  StimulatorMAState;
@property (NS_NONATOMIC_IOSONLY) NTCallibriStimulationParams* _Nullable  StimulatorParam;
@property (NS_NONATOMIC_IOSONLY) NTCallibriMotionAssistantParams* _Nullable  MotionAssistantParam;
@property (NS_NONATOMIC_IOSONLY) NTCallibriMotionCounterParam* _Nullable  MotionCounterParam;
@property (NS_NONATOMIC_IOSONLY, readonly) uint  MotionCounter;
@property (NS_NONATOMIC_IOSONLY, readonly) NTCallibriColorType  Color;
@property (NS_NONATOMIC_IOSONLY) NTCallibriSignalType SignalType;
@property (NS_NONATOMIC_IOSONLY, readonly) NTSensorSamplingFrequency SamplingFrequencyResp;
@property (NS_NONATOMIC_IOSONLY, readonly) bool MEMSCalibrateState;
@property (NS_NONATOMIC_IOSONLY, readonly) NTCallibriElectrodeState ElectrodeState;

@property (NS_NONATOMIC_IOSONLY, readwrite) NTSensorSamplingFrequency SamplingFrequency;
@property (NS_NONATOMIC_IOSONLY, readwrite) NTSensorGain Gain;
@property (NS_NONATOMIC_IOSONLY, readwrite) NTSensorDataOffset DataOffset;
@property (NS_NONATOMIC_IOSONLY, readwrite) NTSensorFirmwareMode FirmwareMode;

- (BOOL) isSupportedFilter:(NTSensorFilter)filter;

- (void) setMEMSDataCallback:(void (^_Nullable)(NSArray<NTMEMSData*>*_Nonnull))callback;            // mems
- (void) setQuaternionDataCallback:(void (^_Nullable)(NSArray<NTQuaternionData*>*_Nonnull))callback;        // callibri

- (void) setSignalCallback:(void (^_Nullable)(NSArray<NTCallibriSignalData*>*_Nonnull))callback;
- (void) setRespirationCallback:(void (^_Nullable)(NSArray<NTCallibriRespirationData*>*_Nonnull))callback;
- (void) setElectrodeStateCallback:(void (^_Nullable)(NTCallibriElectrodeState))callback;
- (void) setEnvelopeDataCallback:(void (^_Nullable)(NSArray<NTCallibriEnvelopeData*>*_Nonnull))callback;

@end
#endif /* NTCallibri_h */
