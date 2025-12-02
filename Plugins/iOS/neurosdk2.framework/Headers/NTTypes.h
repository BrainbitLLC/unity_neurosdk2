//
//  NTTypes.h
//  neurosdk
//
//  Created by Aseatari on 29.08.2022.
//

#ifndef NTTypes_h
#define NTTypes_h

@interface NTOpStatus
@end

typedef NS_ENUM (UInt8, NTSensorFamily) {
    NTSensorFamilyUnknown = 0,

    NTSensorFamilyLECallibri = 1,
    NTSensorFamilyLEKolibri = 2,


    NTSensorFamilyLEBrainBit = 3,


    NTSensorFamilyLEBrainBitBlack = 4,



    NTSensorFamilyLEHeadPhones2 = 6,
 

    NTSensorFamilyLEHeadband = 11,

    NTSensorFamilyLEEarBuds = 12,


    NTSensorFamilyLENeuroEEG = 14,


    NTSensorFamilyLEBrainBit2 = 18,
    NTSensorFamilyLEBrainBitFlex = 19, 
    NTSensorFamilyLEBrainBitPro = 20

};


@interface NTSensorVersion : NSObject
@property (nonatomic) UInt32 FwMajor;
@property (nonatomic) UInt32 FwMinor;
@property (nonatomic) UInt32 FwPatch;

@property (nonatomic) UInt32 HwMajor;
@property (nonatomic) UInt32 HwMinor;
@property (nonatomic) UInt32 HwPatch;

@property (nonatomic, readonly) UInt32 ExtMajor;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end
@interface NTSensorInfo : NSObject
@property (nonatomic) enum NTSensorFamily SensFamily;
@property (nonatomic) UInt8 SensModel;
@property (nonatomic, copy) NSString *_Nonnull Name;
@property (nonatomic, copy) NSString *_Nonnull Address;
@property (nonatomic, copy) NSString *_Nonnull SerialNumber;
@property (nonatomic) BOOL PairingRequared;
@property (nonatomic) SInt16 RSSI;

- (nonnull instancetype)init NS_UNAVAILABLE;
- (nonnull instancetype)initWithSensFamily:(enum NTSensorFamily)sensFamily sensModel:(UInt8)sensModel name:(NSString *_Nonnull)name address:(NSString *_Nonnull)address serialNumber:(NSString *_Nonnull)serialNumber pairingRequared:(BOOL)pairingRequared rssi:(SInt16)rssi NS_DESIGNATED_INITIALIZER;

@end

typedef NS_ENUM (UInt8, NTSensorFeature)
{
    NTSensorFeatureSignal,
    NTSensorFeatureMEMS,
    NTSensorFeatureCurrentStimulator,
    NTSensorFeatureRespiration,
    NTSensorFeatureResist,
    NTSensorFeatureFPG,
    NTSensorFeatureEnvelope,
    NTSensorFeaturePhotoStimulator,
    NTSensorFeatureAcousticStimulator,
    NTSensorFeatureFlashCard,
    NTSensorFeatureLedChannels,
    NTSensorFeatureSignalWithResist,
    NTSensorFeaturePulseOximeter
};

typedef NS_ENUM (UInt8, NTSensorFirmwareMode){
    NTSensorFirmwareModeBootloader,
    NTSensorFirmwareModeApplication
};

typedef NS_ENUM (UInt8, NTSensorCommand)
{
    NTSensorCommandStartSignal,
    NTSensorCommandStopSignal,
    NTSensorCommandStartResist,
    NTSensorCommandStopResist,
    NTSensorCommandStartMEMS,
    NTSensorCommandStopMEMS,
    NTSensorCommandStartRespiration,
    NTSensorCommandStopRespiration,
    NTSensorCommandStartCurrentStimulation,
    NTSensorCommandStopCurrentStimulation,
    NTSensorCommandEnableMotionAssistant,
    NTSensorCommandDisableMotionAssistant,
    NTSensorCommandFindMe,
    NTSensorCommandStartAngle,
    NTSensorCommandStopAngle,
    NTSensorCommandCalibrateMEMS,
    NTSensorCommandResetQuaternion,
    NTSensorCommandStartEnvelope,
    NTSensorCommandStopEnvelope,
    NTSensorCommandResetMotionCounter,
    NTSensorCommandCalibrateStimulation,
    NTSensorCommandIdle,
    NTSensorCommandPowerDown,
    NTSensorCommandStartFPG,
    NTSensorCommandStopFPG,
    NTSensorCommandStartSignalAndResist,
    NTSensorCommandStopSignalAndResist,
    NTSensorCommandStartPhotoStimulation,
    NTSensorCommandStopPhotoStimulation,
    NTSensorCommandStartAcousticStimulation,
    NTSensorCommandStopAcousticStimulation,
    NTSensorCommandFileSystemEnable,
    NTSensorCommandFileSystemDisable,
    NTSensorCommandFileSystemStreamClose,
    NTSensorCommandStartCalibrateSignal,
    NTSensorCommandStopCalibrateSignal,
    NTSensorCommandPhotoStimEnable,
    NTSensorCommandPhotoStimDisable,
    NTSensorCommandStartPulseOximeter,
    NTSensorCommandStopPulseOximeter
};

typedef NS_ENUM (UInt8, NTSensorParameter) {
    NTSensorParameterName,
    NTSensorParameterState,
    NTSensorParameterAddress,
    NTSensorParameterSerialNumber,
    NTSensorParameterHardwareFilterState,
    NTSensorParameterFirmwareMode,
    NTSensorParameterSamplingFrequency,
    NTSensorParameterGain,
    NTSensorParameterOffset,
    NTSensorParameterExternalSwitchState,
    NTSensorParameterADCInputState,
    NTSensorParameterAccelerometerSens,
    NTSensorParameterGyroscopeSens,
    NTSensorParameterStimulatorAndMAState,
    NTSensorParameterStimulatorParamPack,
    NTSensorParameterMotionAssistantParamPack,
    NTSensorParameterFirmwareVersion,
    NTSensorParameterMEMSCalibrationStatus,
    NTSensorParameterMotionCounterParamPack,
    NTSensorParameterMotionCounter,
    NTSensorParameterBattPower,
    NTSensorParameterSensorFamily,
    NTSensorParameterSensorMode,
    NTSensorParameterIrAmplitude,
    NTSensorParameterRedAmplitude,
    NTSensorParameterEnvelopeAvgWndSz,
    NTSensorParameterEnvelopeDecimation,
    NTSensorParameterSamplingFrequencyResist,
    NTSensorParameterSamplingFrequencyMEMS,
    NTSensorParameterSamplingFrequencyFPG,
    NTSensorParameterAmplifier,
    NTSensorParameterSensorChannels,
    NTSensorParameterSamplingFrequencyResp,
    NTSensorParameterSurveyId,
    NTSensorParameterFileSystemStatus,
    NTSensorParameterFileSystemDiskInfo,
    NTSensorParameterReferentsShort,
    NTSensorParameterReferentsGround,
    NTSensorParameterSamplingFrequencyEnvelope,
    NTSensorParameterChannelConfiguration,
    NTSensorParameterElectrodeState,
    NTSensorParameterChannelResistConfiguration,
    NTSensorParameterBattVoltage,
    NTSensorParameterPhotoStimTimeDefer,
    NTSensorParameterPhotoStimSyncState,
    NTSensorParameterSensorPhotoStim,
    NTSensorParameterStimMode,
    NTSensorParameterLedChannels,
    NTSensorParameterLedState,
    NTSensorParameterPulseOximeterParamPack,
    NTSensorParameterPulseOximeterState,
    NTSensorParameterSamplingFrequencyPulseOximeter
};

typedef NS_ENUM (UInt8, NTSensorParamAccess) {
    NTSensorParamAccessRead,
    NTSensorParamAccessReadWrite,
    NTSensorParamAccessReadNotify
};

@interface NTParameterInfo : NSObject
@property (nonatomic) enum NTSensorParameter Param;
@property (nonatomic) enum NTSensorParamAccess ParamAccess;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

typedef NS_ENUM (UInt8, NTSensorState) {
    NTSensorStateInRange,
    NTSensorStateOutOfRange
};

typedef NS_ENUM (UInt8, NTSensorFilter) {
    NTSensorFilterHPFBwhLvl1CutoffFreq1Hz,
    NTSensorFilterHPFBwhLvl1CutoffFreq5Hz,
    NTSensorFilterBSFBwhLvl2CutoffFreq45_55Hz,
    NTSensorFilterBSFBwhLvl2CutoffFreq55_65Hz,
    NTSensorFilterHPFBwhLvl2CutoffFreq10Hz,
    NTSensorFilterLPFBwhLvl2CutoffFreq400Hz,
    NTSensorFilterHPFBwhLvl2CutoffFreq80Hz,
    NTSensorFilterUnknown
};

typedef NS_ENUM (UInt8, NTSensorSamplingFrequency) {
    NTSensorSamplingFrequencyHz10,
    NTSensorSamplingFrequencyHz20,
    NTSensorSamplingFrequencyHz100,
    NTSensorSamplingFrequencyHz125,
    NTSensorSamplingFrequencyHz250,
    NTSensorSamplingFrequencyHz500,
    NTSensorSamplingFrequencyHz1000,
    NTSensorSamplingFrequencyHz2000,
    NTSensorSamplingFrequencyHz4000,
    NTSensorSamplingFrequencyHz8000,
    NTSensorSamplingFrequencyHz10000,
	NTSensorSamplingFrequencyHz12000,
	NTSensorSamplingFrequencyHz16000,
	NTSensorSamplingFrequencyHz24000,
	NTSensorSamplingFrequencyHz32000,
	NTSensorSamplingFrequencyHz48000,
	NTSensorSamplingFrequencyHz64000,
    NTSensorSamplingFrequencyHz50,
    NTSensorSamplingFrequencyUnsupported = 0xFF
};

typedef NS_ENUM (UInt8, NTSensorGain){
    NTSensorGain1,
    NTSensorGain2,
    NTSensorGain3,
    NTSensorGain4,
    NTSensorGain6,
    NTSensorGain8,
    NTSensorGain12,
    NTSensorGain5,
	NTSensorGain2x,
	NTSensorGain4x,
    NTSensorGainUnsupported
};

typedef NS_ENUM (UInt8, NTSensorDataOffset) {
    NTSensorDataOffset0 = 0x00,
    NTSensorDataOffset1 = 0x01,
    NTSensorDataOffset2 = 0x02,
    NTSensorDataOffset3 = 0x03,
    NTSensorDataOffset4 = 0x04,
    NTSensorDataOffset5 = 0x05,
    NTSensorDataOffset6 = 0x06,
    NTSensorDataOffset7 = 0x07,
    NTSensorDataOffset8 = 0x08,
    NTSensorDataOffsetUnsupported = 0xFF
};

typedef NS_ENUM (UInt8, NTSensorExternalSwitchInput) {
    NTSensorExternalSwitchInputElectrodesRespUSB,
    NTSensorExternalSwitchInputElectrodes,
    NTSensorExternalSwitchInputUSB,
    NTSensorExternalSwitchInputRespUSB,
    NTSensorExternalSwitchInputShort,
    NTSensorExternalSwitchInputUnknown = 0xFF
};

typedef NS_ENUM (UInt8, NTSensorADCInput) {
    NTSensorADCInputElectrodes,
    NTSensorADCInputShort,
    NTSensorADCInputTest,
    NTSensorADCInputResistance
};


typedef NS_ENUM (UInt8, NTSensorAccelerometerSensitivity) {
    NTSensorAccelerometerSensitivity2g,
    NTSensorAccelerometerSensitivity4g,
    NTSensorAccelerometerSensitivity8g,
    NTSensorAccelerometerSensitivity16g,
    NTSensorAccelerometerSensitivityUnsupported
};

typedef NS_ENUM (UInt8, NTSensorGyroscopeSensitivity) {
    NTSensorGyroscopeSensitivity250Grad,
    NTSensorGyroscopeSensitivity500Grad,
    NTSensorGyroscopeSensitivity1000Grad,
    NTSensorGyroscopeSensitivity2000Grad,
    NTSensorGyroscopeSensitivityUnsupported
};




typedef NS_ENUM (UInt8, NTCallibriStimulatorState) {
    NTCallibriStimulatorStateNoParams = 0,
    NTCallibriStimulatorStateDisabled = 1,
    NTCallibriStimulatorStateEnabled = 2,
    NTCallibriStimulatorStateUnsupported = 0xFF
};

@interface NTCallibriStimulatorMAState : NSObject
    @property (nonatomic) enum NTCallibriStimulatorState stimulatorState;
    @property (nonatomic) enum NTCallibriStimulatorState MAState;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

/// <summary>
/// Stimulator parameters
/// Limitations:
/// (Current * Frequency * PulseWidth / 100) <= 2300 uA
/// </summary>
@interface NTCallibriStimulationParams : NSObject
    /// <summary>
    /// Stimulus amplitude in  mA. 1..100
    /// </summary>
@property (nonatomic) UInt8 Current;
    /// <summary>
    /// Duration of the stimulating pulse by us. 20..460
    /// </summary>
@property (nonatomic) UInt16 PulseWidth;
    /// <summary>
    /// Frequency of stimulation impulses by Hz. 1..200.
    /// </summary>
@property (nonatomic) UInt8 Frequency;
    /// <summary>
    /// Maximum stimulation time by ms. 0...65535.
    /// Zero is infinitely.
    /// </summary>
@property (nonatomic) UInt16 StimulusDuration;

- (nonnull instancetype)init NS_UNAVAILABLE;
- (nonnull instancetype)initWithCurrent:(UInt8)current pulseWidth:(UInt16)pulseWidth frequency:(UInt8)frequency stimulusDuration:(UInt16)stimulusDuration NS_DESIGNATED_INITIALIZER;

@end

typedef NS_ENUM (UInt8, NTCallibriMotionAssistantLimb) {
    NTCallibriMotionAssistantLimbRightLeg = 0,
    NTCallibriMotionAssistantLimbLeftLeg = 1,
    NTCallibriMotionAssistantLimbRightArm = 2,
    NTCallibriMotionAssistantLimbLeftArm = 3,
    NTCallibriMotionAssistantLimbUnsupported = 0xFF
};

@interface NTCallibriMotionAssistantParams : NSObject
@property (nonatomic) UInt8 GyroStart;
@property (nonatomic) UInt8 GyroStop;
@property (nonatomic) enum NTCallibriMotionAssistantLimb Limb;
    /// <summary>
    /// multiple of 10. This means that the device is using the (MinPauseMs / 10) value.;
    /// Correct values: 10, 20, 30, 40 ...
    /// </summary>
@property (nonatomic) UInt8 MinPauseMs;

- (nonnull instancetype)init NS_UNAVAILABLE;
- (nonnull instancetype)initWithGyroStart:(UInt8)gyroStart gyroStop:(UInt8)gyroStop limb:(enum NTCallibriMotionAssistantLimb)limb minPauseMs:(UInt8)minPauseMs NS_DESIGNATED_INITIALIZER;

@end

@interface NTCallibriMotionCounterParam : NSObject
    /// <summary>
    /// Insense threshold mg. 0..500
    /// </summary>
@property (nonatomic) UInt16 InsenseThresholdMG;
    /// <summary>
    /// Algorithm insense threshold in time (in samples with the MEMS sampling rate) 0..500
    /// </summary>
@property (nonatomic) UInt16 InsenseThresholdSample;

- (nonnull instancetype)init NS_UNAVAILABLE;
- (nonnull instancetype)initWithInsenseThresholdMG:(UInt16)insenseThresholdMG insenseThresholdSample:(UInt16)insenseThresholdSample NS_DESIGNATED_INITIALIZER;

@end

@interface NTCallibriSignalData : NSObject
@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) NSArray<NSNumber*>* _Nonnull Samples;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

@interface NTCallibriRespirationData : NSObject
@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) NSArray<NSNumber*>* _Nonnull Samples;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

typedef NS_ENUM (UInt8, NTCallibriElectrodeState) {
    NTCallibriElectrodeStateNormal,
    NTCallibriElectrodeStateHighResistance,
    NTCallibriElectrodeStateDetached
};
@interface NTQuaternionData : NSObject
@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) Float32 W;
@property (nonatomic, readonly) Float32 X;
@property (nonatomic, readonly) Float32 Y;
@property (nonatomic, readonly) Float32 Z;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

@interface NTCallibriEnvelopeData : NSObject
@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) NSNumber*_Nonnull Sample;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

struct Point3D {
    double X;
    double Y;
    double Z;
};

@interface NTMEMSData : NSObject
@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) struct Point3D Accelerometer;
@property (nonatomic, readonly) struct Point3D Gyroscope;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

typedef NS_ENUM (UInt8, NTGenCurrent) {
    NTGenCurrentGenCurr0uA = 0,
    NTGenCurrentGenCurr6nA = 1,
    NTGenCurrentGenCurr12nA = 2,
    NTGenCurrentGenCurr18nA = 3,
    NTGenCurrentGenCurr24nA = 4,
    NTGenCurrentGenCurr6uA = 5,
    NTGenCurrentGenCurr24uA = 6,
    NTGenCurrentGenUnsupported = 0xFF
};

typedef NS_ENUM (UInt8, NTBrainBit2ChannelMode) {
    NTBrainBit2ChannelModeShort = 0,
    NTBrainBit2ChannelModeNormal = 1
};

@interface NTBrainBit2AmplifierParam : NSObject
@property (nonatomic) NSMutableArray<NSNumber*>* _Nonnull ChSignalMode;
@property (nonatomic) NSMutableArray<NSNumber*>* _Nonnull ChResistUse;
@property (nonatomic) NSMutableArray<NSNumber*>* _Nonnull ChGain;
@property (nonatomic) enum NTGenCurrent Current;

- (nonnull instancetype)init;

@end

@interface NTResistRefChannelsData : NSObject
@property (nonatomic) UInt32 PackNum;
@property (nonatomic) NSMutableArray<NSNumber*>* _Nonnull Samples;
@property (nonatomic) NSMutableArray<NSNumber*>* _Nonnull Referents;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

typedef NS_ENUM (UInt8, NTIrAmplitude) {
    NTIrAmplitudeIrAmp0 = 0,
    NTIrAmplitudeIrAmp14 = 1,
    NTIrAmplitudeIrAmp28 = 2,
    NTIrAmplitudeIrAmp42 = 3,
    NTIrAmplitudeIrAmp56 = 4,
    NTIrAmplitudeIrAmp70 = 5,
    NTIrAmplitudeIrAmp84 = 6,
    NTIrAmplitudeIrAmp100 = 7,
    NTIrAmplitudeIrAmpUnsupported = 0xFF
};

typedef NS_ENUM (UInt8, NTRedAmplitude) {
    NTRedAmplitudeRedAmp0 = 0,
    NTRedAmplitudeRedAmp14 = 1,
    NTRedAmplitudeRedAmp28 = 2,
    NTRedAmplitudeRedAmp42 = 3,
    NTRedAmplitudeRedAmp56 = 4,
    NTRedAmplitudeRedAmp70 = 5,
    NTRedAmplitudeRedAmp84 = 6,
    NTRedAmplitudeRedAmp100 = 7,
    NTRedAmplitudeRedAmpUnsupported = 0xFF
};

@interface NTBrainBitSignalData : NSObject

@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) UInt8 Marker;
@property (nonatomic, readonly) NSNumber*_Nonnull O1;
@property (nonatomic, readonly) NSNumber*_Nonnull O2;
@property (nonatomic, readonly) NSNumber*_Nonnull T3;
@property (nonatomic, readonly) NSNumber*_Nonnull T4;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end
@interface NTBrainBitResistData : NSObject
    @property (nonatomic, readonly) NSNumber*_Nonnull O1;
    @property (nonatomic, readonly) NSNumber*_Nonnull O2;
    @property (nonatomic, readonly) NSNumber*_Nonnull T3;
    @property (nonatomic, readonly) NSNumber*_Nonnull T4;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end


@interface NTHeadbandSignalData : NSObject
@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) UInt8 Marker;
@property (nonatomic, readonly) NSNumber*_Nonnull O1;
@property (nonatomic, readonly) NSNumber*_Nonnull O2;
@property (nonatomic, readonly) NSNumber*_Nonnull T3;
@property (nonatomic, readonly) NSNumber*_Nonnull T4;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end
@interface NTHeadbandResistData : NSObject
@property (nonatomic, readonly) NSNumber*_Nonnull O1;
@property (nonatomic, readonly) NSNumber*_Nonnull O2;
@property (nonatomic, readonly) NSNumber*_Nonnull T3;
@property (nonatomic, readonly) NSNumber*_Nonnull T4;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end





@interface NTHeadphones2SignalData : NSObject

@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) UInt8 Marker;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch1;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch2;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch3;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch4;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

@interface NTHeadphones2ResistData : NSObject
@property (nonatomic, readonly) UInt32 PackNum;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch1;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch2;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch3;
@property (nonatomic, readonly) NSNumber*_Nonnull Ch4;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end

@interface NTHeadphones2AmplifierParam : NSObject
@property (nonatomic) BOOL  ChSignalUse1;
@property (nonatomic) BOOL  ChSignalUse2;
@property (nonatomic) BOOL  ChSignalUse3;
@property (nonatomic) BOOL  ChSignalUse4;

@property (nonatomic) BOOL  ChResistUse1;
@property (nonatomic) BOOL  ChResistUse2;
@property (nonatomic) BOOL  ChResistUse3;
@property (nonatomic) BOOL  ChResistUse4;

@property (nonatomic) NTSensorGain ChGain1;
@property (nonatomic) NTSensorGain ChGain2;
@property (nonatomic) NTSensorGain ChGain3;
@property (nonatomic) NTSensorGain ChGain4;

@property (nonatomic) NTGenCurrent Current;

- (nonnull instancetype)init NS_UNAVAILABLE;
- (nonnull instancetype)initWithChSignalUse1:(BOOL)chSignalUse1 chSignalUse2:(BOOL)chSignalUse2 chSignalUse3:(BOOL)chSignalUse3 chSignalUse4:(BOOL)chSignalUse4  chResistUse1:(BOOL)chResistUse1 chResistUse2:(BOOL)chResistUse2 chResistUse3:(BOOL)chResistUse3 chResistUse4:(BOOL)chResistUse4  chGain1:(NTSensorGain)chGain1 chGain2:(NTSensorGain)chGain2 chGain3:(NTSensorGain)chGain3 chGain4:(NTSensorGain)chGain4 current:(NTGenCurrent)current NS_DESIGNATED_INITIALIZER;

@end


typedef NS_ENUM (UInt8, NTSensorAmpMode)
{
    NTSensorAmpModeInvalid,
    NTSensorAmpModePowerDown,
    NTSensorAmpModeIdle,
    NTSensorAmpModeSignal,
    NTSensorAmpModeResist,
    NTSensorAmpModeSignalResist
};


@interface NTFPGData : NSObject
@property (nonatomic, readonly) UInt32  PackNum;
@property (nonatomic, readonly) NSNumber*_Nonnull IrAmplitude;
@property (nonatomic, readonly) NSNumber*_Nonnull RedAmplitude;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end


typedef NS_ENUM (UInt8, NTCallibriColorType)
{
    NTCallibriColorTypeRed,
    NTCallibriColorTypeYellow,
    NTCallibriColorTypeBlue,
    NTCallibriColorTypeWhite,

    NTCallibriColorTypeUnknown
};

typedef NS_ENUM (UInt8, NTCallibriSignalType)
{
    NTCallibriSignalTypeEEG = 0,
    NTCallibriSignalTypeEMG = 1,
    NTCallibriSignalTypeECG = 2,
    NTCallibriSignalTypeEDA = 3,// GSR
    NTCallibriSignalTypeStrainGaugeBreathing = 4,
    NTCallibriSignalTypeImpedanceBreathing = 5,
    NTCallibriSignalTypeTenzoBreathing = 6,
    NTCallibriSignalTypeUnknown = 7
};




typedef NS_ENUM(UInt8, NTEEGChannelMode)
{
    NTEEGChannelModeOff,
    NTEEGChannelModeShorted,
    NTEEGChannelModeSignalResist,
    NTEEGChannelModeSignal,
    NTEEGChannelModeTest
};

typedef NS_ENUM (UInt8, NTSensorFSStatus)
{
    NTSensorFSStatusOK,
    NTSensorFSStatusNoInit,
    NTSensorFSStatusNoDisk,
    NTSensorFSStatusProtect
};

typedef NS_ENUM (UInt8, NTSensorFSIOStatus)
{
    NTSensorFSIOStatusNoError,
    NTSensorFSIOStatusIOError,
    NTSensorFSIOStatusTimeout
};

typedef NS_ENUM (UInt8, NTSensorFSStreamStatus)
{
    NTSensorFSStreamStatusClosed,
    NTSensorFSStreamStatusWrite,
    NTSensorFSStreamStatusRead
};

@interface NTSensorDiskInfo : NSObject
@property (nonatomic, readonly) UInt64 TotalSize;
@property (nonatomic, readonly) UInt64 FreeSize;
@end

typedef NS_ENUM (UInt8, NTEEGRefMode)
{
    NTEEGRefModeHeadTop = 1,
    NTEEGRefModeA1A2
};

@interface NTNeuroEEGAmplifierParam : NSObject
@property (nonatomic) BOOL ReferentResistMesureAllow;
@property (nonatomic) enum NTSensorSamplingFrequency Frequency;
@property (nonatomic) enum NTEEGRefMode ReferentMode;
@property (nonatomic) NSMutableArray<NSNumber*>* _Nonnull ChannelMode;
@property (nonatomic) NSMutableArray<NSNumber*>* _Nonnull ChannelGain;
@property (nonatomic) BOOL UseDiffAsRespiration;
- (nonnull instancetype)init;
@end

@interface NTNeuroEEGFSStatus : NSObject
@property (nonatomic) NTSensorFSStatus Status;
@property (nonatomic) NTSensorFSIOStatus IOStatus;
@property (nonatomic) NTSensorFSStreamStatus StreamStatus;
@property (nonatomic) BOOL AutosaveSignal;
@end


@interface NTResistChannelsData : NSObject
@property (nonatomic) UInt32 PackNum;
@property (nonatomic) NSNumber*_Nonnull A1;
@property (nonatomic) NSNumber*_Nonnull A2;
@property (nonatomic) NSNumber*_Nonnull Ref;
@property (nonatomic) NSNumber*_Nonnull Bias;
@property (nonatomic) NSArray<NSNumber*>* _Nonnull Values;
@end

@interface NTSensorFileData : NSObject
@property (nonatomic, readonly) UInt32 OffsetStart;
@property (nonatomic, readonly) UInt32 DataAmount;
@property (nonatomic, readonly) NSData*_Nonnull Data;
@end

@interface NTSensorFileInfo : NSObject
@property (nonatomic, readonly) NSString*_Nonnull FileName;//char FileName[FILE_NAME_MAX_LEN];
@property (nonatomic, readonly) UInt32 FileSize;
@property (nonatomic, readonly) UInt16 ModifiedYear;
@property (nonatomic, readonly) UInt8 ModifiedMonth;
@property (nonatomic, readonly) UInt8 ModifiedDayOfMonth;
@property (nonatomic, readonly) UInt8 ModifiedHour;
@property (nonatomic, readonly) UInt8 ModifiedMin;
@property (nonatomic, readonly) UInt8 ModifiedSec;
@property (nonatomic, readonly) UInt8 Attribute;
@end


@interface NTSignalChannelsData : NSObject
@property (nonatomic) UInt32 PackNum;
@property (nonatomic) UInt8 Marker;
@property (nonatomic) NSArray<NSNumber*>* _Nonnull Samples;
@end


typedef NS_ENUM (UInt8, NTEEGChannelType)
{
    NTEEGChannelTypeSingleA1,
    NTEEGChannelTypeSingleA2,
    NTEEGChannelTypeDifferential,
	NTEEGChannelTypeRef
};

typedef NS_ENUM (UInt8, NTEEGChannelId)
{
    NTEEGChannelIdUnknown,
    NTEEGChannelIdO1,
    NTEEGChannelIdP3,
    NTEEGChannelIdC3,
    NTEEGChannelIdF3,
    NTEEGChannelIdFp1,
    NTEEGChannelIdT5,
    NTEEGChannelIdT3,
    NTEEGChannelIdF7,

    NTEEGChannelIdF8,
    NTEEGChannelIdT4,
    NTEEGChannelIdT6,
    NTEEGChannelIdFp2,
    NTEEGChannelIdF4,
    NTEEGChannelIdC4,
    NTEEGChannelIdP4,
    NTEEGChannelIdO2,

    NTEEGChannelIdD1,
    NTEEGChannelIdD2,
    NTEEGChannelIdOZ,
    NTEEGChannelIdPZ,
    NTEEGChannelIdCZ,
    NTEEGChannelIdFZ,
    NTEEGChannelIdFpZ,
    NTEEGChannelIdD3
};

@interface NTEEGChannelInfo : NSObject
@property (nonatomic, readonly) NTEEGChannelId Id;
@property (nonatomic, readonly) NTEEGChannelType ChType;
@property (nonatomic, readonly) NSString*_Nonnull Name;
@property (nonatomic, readonly) UInt8  Num;

- (nonnull instancetype)init NS_UNAVAILABLE;

@end


#endif /* NTTypes_h */
