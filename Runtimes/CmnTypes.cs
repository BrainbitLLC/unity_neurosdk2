using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace NeuroSDK
{

    #region CmnType
    internal class SdkLibConst
    {
        public const int ErrMsgLen = 512;
        public const int SensorNameLen = 256;
        public const int SensorAdrLen = 128;
        public const int SensorSNLen = 128;
        public const int SensorChannelNameLen = 8;
        public const int FileNameMaxLen = 64;
        public const int NeuroEEGMaxChCount = 24;
        public const int NeuroBAMMaxChCount = 8;
        
        public const int BrainBit2MaxChCount = 8;
        
    }

    /// <summary>
    /// Operation status
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct OpStatus
    {
        [MarshalAs(UnmanagedType.I1)]
        public bool Success;
        [MarshalAs(UnmanagedType.U4)]
        public uint Error;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SdkLibConst.ErrMsgLen)]
        public string ErrorMsg;
    }
    /// <summary>
    /// Sensor Family
    /// </summary>
    public enum SensorFamily : byte
    {
        SensorUnknown = 0,
        SensorLECallibri = 1,
        SensorLEKolibri = 2,
        SensorLEBrainBit = 3,
        SensorLEBrainBitBlack = 4,
        
        
        
        
        
        
        
        
        SensorLEBrainBit2 = 18,
        SensorLEBrainBitPro = 19,
        SensorLEBrainBitFlex = 20,
        
    }

    /// <summary>
    /// Sensor information
    /// </summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SensorInfo
    {
        [MarshalAs(UnmanagedType.U1)]
        public SensorFamily SensFamily;
        [MarshalAs(UnmanagedType.U1)]
        public byte SensModel;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SdkLibConst.SensorNameLen)]
        public string Name;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SdkLibConst.SensorAdrLen)]
        public string Address;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SdkLibConst.SensorSNLen)]
        public string SerialNumber;
        [MarshalAs(UnmanagedType.I1)]
        public bool PairingRequired;
        [MarshalAs(UnmanagedType.I2)]
        public short RSSI;
    }
    /// <summary>
    /// Sensor Feature
    /// </summary>
    public enum SensorFeature : sbyte
    {
        FeatureSignal,
        FeatureMEMS,
        FeatureCurrentStimulator,
        FeatureRespiration,
        FeatureResist,
        FeatureFPG,
        FeatureEnvelope,
        FeaturePhotoStimulator,
        FeatureAcousticStimulator,
        FeatureFlashCard
    }
    /// <summary>
    /// Sensor firmware mode
    /// </summary>
    public enum SensorFirmwareMode : sbyte
    {
        ModeBootloader,
        ModeApplication
    }
    /// <summary>
    /// Sensor commands
    /// </summary>
    public enum SensorCommand : sbyte
    {
        CommandStartSignal,
        CommandStopSignal,
        CommandStartResist,
        CommandStopResist,
        CommandStartMEMS,
        CommandStopMEMS,
        CommandStartRespiration,
        CommandStopRespiration,
        CommandStartCurrentStimulation,
        CommandStopCurrentStimulation,
        CommandEnableMotionAssistant,
        CommandDisableMotionAssistant,
        CommandFindMe,
        CommandStartAngle,
        CommandStopAngle,
        CommandCalibrateMEMS,
        CommandResetQuaternion,
        CommandStartEnvelope,
        CommandStopEnvelope,
        CommandResetMotionCounter,
        CommandCalibrateStimulation,
        CommandIdle,
        CommandPowerDown,
        CommandStartFPG,
        CommandStopFPG,
        CommandStartSignalAndResist,
        CommandStopSignalAndResist,
        CommandStartPhotoStimulation,
        CommandStopPhotoStimulation,
        CommandStartAcousticStimulation,
        CommandStopAcousticStimulation,
        CommandFileSystemEnable,
	    CommandFileSystemDisable,
	    CommandFileSystemStreamClose,
        CommandStartCalibrateSignal,
	    CommandStopCalibrateSignal,
        CommandPhotoStimEnable,
	    CommandPhotoStimDisable
    }
    /// <summary>
    /// Sensor parameters
    /// </summary>
    public enum SensorParameter : sbyte
    {
        ParameterName,
        ParameterState,
        ParameterAddress,
        ParameterSerialNumber,
        ParameterHardwareFilterState,
        ParameterFirmwareMode,
        ParameterSamplingFrequency,
        ParameterGain,
        ParameterOffset,
        ParameterExternalSwitchState,
        ParameterADCInputState,
        ParameterAccelerometerSens,
        ParameterGyroscopeSens,
        ParameterStimulatorAndMAState,
        ParameterStimulatorParamPack,
        ParameterMotionAssistantParamPack,
        ParameterFirmwareVersion,
        ParameterMEMSCalibrationStatus,
        ParameterMotionCounterParamPack,
        ParameterMotionCounter,
        ParameterBattPower,
        ParameterSensorFamily,
        ParameterSensorMode,
        ParameterIrAmplitude,
        ParameterRedAmplitude,
        ParameterEnvelopeAvgWndSz,
        ParameterEnvelopeDecimation,
        ParameterSamplingFrequencyResist,
        ParameterSamplingFrequencyMEMS,
        ParameterSamplingFrequencyFPG,
        ParameterAmplifier,
        ParameterSensorChannels,
        ParameterSamplingFrequencyResp,
        ParameterSurveyId,
	    ParameterFileSystemStatus,
        ParameterFileSystemDiskInfo,
        ParameterReferentsShort,
		ParameterReferentsGround,
        ParameterSamplingFrequencyEnvelope,
	    ParameterChannelConfiguration,
	    ParameterElectrodeState,
        ParameterChannelResistConfiguration,
        ParameterBattVoltage,
	    ParameterPhotoStimTimeDefer,
        ParameterPhotoStimSyncState,
	    ParameterSensorPhotoStim,
	    ParameterPhotoStimMode
    }
    /// <summary>
    /// Sensor parameter access
    /// </summary>
    public enum SensorParamAccess : sbyte
    {
        ParamAccessRead,
        ParamAccessReadWrite,
        ParamAccessReadNotify,
        ParamAccessWrite
    }
    /// <summary>
    /// Sensor parameter information
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ParameterInfo
    {
        public SensorParameter Param;
        public SensorParamAccess ParamAccess;
    }
    /// <summary>
    /// Sensor connection state
    /// </summary>
    public enum SensorState : sbyte
    {
        StateInRange,
        StateOutOfRange
    }
    
    /// <summary>
    /// Sensor sampling frequency
    /// </summary>
    public enum SensorSamplingFrequency : sbyte
    {
        FrequencyHz10,
        FrequencyHz20,
        FrequencyHz100,
        FrequencyHz125,
        FrequencyHz250,
        FrequencyHz500,
        FrequencyHz1000,
        FrequencyHz2000,
        FrequencyHz4000,
        FrequencyHz8000,
        FrequencyHz10000,
	    FrequencyHz12000,
	    FrequencyHz16000,
	    FrequencyHz24000,
	    FrequencyHz32000,
	    FrequencyHz48000,
	    FrequencyHz64000,
        FrequencyUnsupported
    }
    /// <summary>
    /// Sensor gain
    /// </summary>
    public enum SensorGain : sbyte
    {
        SensorGain1,
        SensorGain2,
        SensorGain3,
        SensorGain4,
        SensorGain6,
        SensorGain8,
        SensorGain12,
        SensorGain5,
	    SensorGain2x,
	    SensorGain4x,
        SensorGainUnsupported
    }
    /// <summary>
    /// Sensor data offset
    /// </summary>
    public enum SensorDataOffset : byte
    {
        DataOffset0 = 0x00,
        DataOffset1 = 0x01,
        DataOffset2 = 0x02,
        DataOffset3 = 0x03,
        DataOffset4 = 0x04,
        DataOffset5 = 0x05,
        DataOffset6 = 0x06,
        DataOffset7 = 0x07,
        DataOffset8 = 0x08,
        DataOffsetUnsupported = 0xFF
    }
    
        /// <summary>
    /// Sensor Accelerometer Sensitivity
    /// </summary>
    public enum SensorAccelerometerSensitivity : sbyte
    {
        AccSens2g,
        AccSens4g,
        AccSens8g,
        AccSens16g,
        AccSensUnsupported
    }
    /// <summary>
    /// Sensor Gyroscope Sensitivity
    /// </summary>
    public enum SensorGyroscopeSensitivity : sbyte
    {
        GyroSens250Grad,
        GyroSens500Grad,
        GyroSens1000Grad,
        GyroSens2000Grad,
        GyroSensUnsupported
    }

    /// <summary>
    /// Sensor version
    /// </summary>
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorVersion
    {
        [MarshalAs(UnmanagedType.U4)]
        public uint FwMajor;
        [MarshalAs(UnmanagedType.U4)]
        public uint FwMinor;
        [MarshalAs(UnmanagedType.U4)]
        public uint FwPatch;

        [MarshalAs(UnmanagedType.U4)]
        public uint HwMajor;
        [MarshalAs(UnmanagedType.U4)]
        public uint HwMinor;
        [MarshalAs(UnmanagedType.U4)]
        public uint HwPatch;

        [MarshalAs(UnmanagedType.U4)]
        public uint ExtMajor;
    }

        /// <summary>
    /// Sensor filters
    /// </summary>
    public enum SensorFilter : ushort
    {
	    FilterHPFBwhLvl1CutoffFreq1Hz,
	    FilterHPFBwhLvl1CutoffFreq5Hz,
	    FilterBSFBwhLvl2CutoffFreq45_55Hz,
	    FilterBSFBwhLvl2CutoffFreq55_65Hz,
	    FilterHPFBwhLvl2CutoffFreq10Hz,
	    FilterLPFBwhLvl2CutoffFreq400Hz,
	    FilterHPFBwhLvl2CutoffFreq80Hz,
        FilterUnknown = 0xFF
    }

        /// <summary>
    /// Sensor AmpMode
    /// </summary>
    public enum SensorAmpMode : byte
    {
        Invalid,
        PowerDown,
        Idle,
        Signal,
        Resist,
        SignalResist,
        Envelope
    }

    public delegate void SensorAmpModeChanged(ISensor sensor, SensorAmpMode sensorAmpMode);

        /// <summary>
    /// Accelerometer values
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Accelerometer
    {
        public double X;
        public double Y;
        public double Z;
    }
    /// <summary>
    /// Gyroscope values
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Gyroscope
    {
        public double X;
        public double Y;
        public double Z;
    }

    /// <summary>
    /// MEMS Data
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct MEMSData
    {
        public uint PackNum;
        public Accelerometer Accelerometer;
        public Gyroscope Gyroscope;
    }

    public delegate void MEMSDataRecived(ISensor sensor, MEMSData[] data);

        /// <summary>
    /// FPG Data
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct FPGData
    {
        public uint PackNum;
        public double IrAmplitude;
        public double RedAmplitude;
    }

    /// <summary>
    /// Sensor IrAmplitude
    /// </summary>
    public enum IrAmplitude : byte
    {
        IrAmp0 = 0,
        IrAmp14 = 1,
        IrAmp28 = 2,
        IrAmp42 = 3,
        IrAmp56 = 4,
        IrAmp70 = 5,
        IrAmp84 = 6,
        IrAmp100 = 7,
        IrAmpUnsupported = 0xFF
    }
    /// <summary>
    /// Sensor RedAmplitude
    /// </summary>
    public enum RedAmplitude : byte
    {
        RedAmp0 = 0,
        RedAmp14 = 1,
        RedAmp28 = 2,
        RedAmp42 = 3,
        RedAmp56 = 4,
        RedAmp70 = 5,
        RedAmp84 = 6,
        RedAmp100 = 7,
        RedAmpUnsupported = 0xFF
    }

    public delegate void FPGDataRecived(ISensor sensor, FPGData[] data);

        /// <summary>
    /// Representation of possible Callibri/Kolibri/CallibriNext colors
    /// </summary>
    public enum CallibriColorType
    {
        CallibriColorRed,
        CallibriColorYellow,
        CallibriColorBlue,
        CallibriColorWhite,

        CallibriColorUnknown
    };

    /// <summary>
    /// Sensor Callibri Electrode State
    /// </summary>
    public enum CallibriElectrodeState : byte
    {
        ElStNormal,
        ElStHighResistance,
        ElStDetached
    }

    /// <summary>
    /// Sensor external switch input
    /// </summary>
    public enum SensorExternalSwitchInput : byte
    {
        ExtSwInElectrodesRespUSB,
        ExtSwInElectrodes,
        ExtSwInUSB,
        ExtSwInRespUSB,
        ExtSwInShort,
        ExtSwInUnknown = 0xFF
    }

        /// <summary>
    /// Sensor BrainBit Resistance Data
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BrainBitResistData
    {
        public double O1;
        public double O2;
        public double T3;
        public double T4;
    }
    /// <summary>
    /// Sensor BrainBit Signal Data
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BrainBitSignalData
    {
        public uint PackNum;
        public byte Marker;
        public double O1;
        public double O2;
        public double T3;
        public double T4;
    }

    public delegate void BrainBitResistDataRecived(ISensor sensor, BrainBitResistData data);
    public delegate void BrainBitSignalDataRecived(ISensor sensor, BrainBitSignalData[] data);

    
        /// <summary>
    /// Gen Current
    /// </summary>
    public enum GenCurrent : byte
    {
        GenCurr0nA = 0,
        GenCurr6nA = 1,
        GenCurr12nA = 2,
        GenCurr18nA = 3,
        GenCurr24nA = 4,
        GenCurr6uA = 5,
        GenCurr24uA = 6,
        Unsupported = 0xFF
    }

    
    
        /// <summary>
    /// Sensor ADC Input
    /// </summary>
    public enum SensorADCInput : sbyte
    {
        ADCInputElectrodes,
        ADCInputShort,
        ADCInputTest,
        ADCInputResistance
    }
    /// <summary>
    /// Sensor Stimulator State
    /// </summary>
    public enum CallibriStimulatorState : byte
    {
        StimStateNoParams = 0,
        StimStateDisabled = 1,
        StimStateEnabled = 2,
        StimStateUnsupported = 0xFF
    }
    /// <summary>
    /// Sensor Stimulator MA State
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CallibriStimulatorMAState
    {
        public CallibriStimulatorState StimulatorState;
        public CallibriStimulatorState MAState;
    }
    /// <summary>
    /// Stimulator parameters</br>
    /// Limitations:</br>
    /// (Current * Frequency * PulseWidth / 100) <= 2300 uA
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CallibriStimulationParams
    {
        /// <summary>
        /// Stimulus amplitude in  mA. 1..100
        /// </summary>
        public byte Current;
        /// <summary>
        /// Duration of the stimulating pulse by us. 20..460
        /// </summary>
        public ushort PulseWidth;
        /// <summary>
        /// Frequency of stimulation impulses by Hz. 1..200.
        /// </summary>
        public byte Frequency;
        /// <summary>
        /// Maximum stimulation time by ms. 0...65535.</br>
        /// Zero is infinitely.
        /// </summary>
        public ushort StimulusDuration;
    }
    /// <summary>
    /// Callibri sensor MotionAssistant Limb
    /// </summary>
    public enum CallibriMotionAssistantLimb : byte
    {
        MALimbRightLeg = 0,
        MALimbLeftLeg = 1,
        MALimbRightArm = 2,
        MALimbLeftArm = 3,
        MALimbUnsupported = 0xFF
    }
    /// <summary>
    /// Callibri sensor MotionAssistant parameters
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CallibriMotionAssistantParams
    {
        public byte GyroStart;
        public byte GyroStop;
        public CallibriMotionAssistantLimb Limb;
        /// <summary>
        /// multiple of 10. This means that the device is using the (MinPauseMs / 10) value.;</br>
        /// Correct values: 10, 20, 30, 40 ... 
        /// </summary>
        public byte MinPauseMs;
    }
    /// <summary>
    /// Callibri sensor MotionAssistant counter parameters
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CallibriMotionCounterParam
    {
        /// <summary>
        /// Insense threshold mg. 0..500
        /// </summary>
        public ushort InsenseThresholdMG;
        /// <summary>
        /// Algorithm insense threshold in time (in samples with the MEMS sampling rate) 0..500
        /// </summary>
        public ushort InsenseThresholdSample;
    }
    /// <summary>
    /// Sensor Callibri Signal Data
    /// </summary>
    public struct CallibriSignalData
    {
        public uint PackNum;
        public double[] Samples;
    }
    /// <summary>
    /// Sensor Callibri Respiration Data
    /// </summary>
    public struct CallibriRespirationData
    {
        public uint PackNum;
        public double[] Samples;
    }
    /// <summary>
    /// Sensor Callibri Envelope Data
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CallibriEnvelopeData
    {
        public uint PackNum;
        public double Sample;
    }

    public enum CallibriSignalType : sbyte
    {
        EEG = 0,
        EMG = 1,
        ECG = 2,
        EDA = 3,// GSR
        StrainGaugeBreathing = 4,
        ImpedanceBreathing = 5,
        TenzoBreathing = 6,
        Unknown = 7
    }
    /// <summary>
    /// Quaternion Data
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct QuaternionData
    {
        public uint PackNum;
        public float W;
        public float X;
        public float Y;
        public float Z;
    }

    public delegate void CallibriSignalDataRecived(ISensor sensor, CallibriSignalData[] data);
    public delegate void CallibriRespirationDataRecived(ISensor sensor, CallibriRespirationData[] data);
    public delegate void CallibriElectrodeStateChanged(ISensor sensor, CallibriElectrodeState elState);
    public delegate void CallibriEnvelopeDataRecived(ISensor sensor, CallibriEnvelopeData[] data);
    public delegate void QuaternionDataRecived(ISensor sensor, QuaternionData[] data);


        /// <summary>
    /// EEG Channel Type
    /// </summary>
    public enum EEGChannelType : byte
    {
        EEGChTypeSingleA1,
        EEGChTypeSingleA2,
        EEGChTypeDifferential,
        EEGChTypeRef
    }
    /// <summary>
    /// EEG Channel Id
    /// </summary>
    public enum EEGChannelId : byte
    {
        EEGChIdUnknown,
        EEGChIdO1,
        EEGChIdP3,
        EEGChIdC3,
        EEGChIdF3,
        EEGChIdFp1,
        EEGChIdT5,
        EEGChIdT3,
        EEGChIdF7,

        EEGChIdF8,
        EEGChIdT4,
        EEGChIdT6,
        EEGChIdFp2,
        EEGChIdF4,
        EEGChIdC4,
        EEGChIdP4,
        EEGChIdO2,

        EEGChIdD1,
        EEGChIdD2,
        EEGChIdOZ,
        EEGChIdPZ,
        EEGChIdCZ,
        EEGChIdFZ,
        EEGChIdFpZ,
        EEGChIdD3
    }
    /// <summary>
    /// EEG Channel Info
    /// </summary>
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct EEGChannelInfo
    {
        public EEGChannelId Id;
        public EEGChannelType ChType;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SdkLibConst.SensorChannelNameLen)]
        public string Name;
        public byte Num;
    }
    public enum EEGChannelMode : byte {
	    EEGChModeOff,
	    EEGChModeShorted,
	    EEGChModeSignalResist,
	    EEGChModeSignal,
	    EEGChModeTest
    }

    public struct SignalChannelsData {
	    public uint PackNum;
	    public byte Marker;
	    public double[] Samples;
    }

    public struct ResistRefChannelsData {
	    public uint PackNum;
	    public double[] Samples;
	    public double[] Referents;
    }

    
    
    
    
    
    
    public delegate void BrainBit2SignalDataRecived(ISensor sensor, SignalChannelsData[] data);
    public delegate void BrainBit2ResistDataRecived(ISensor sensor, ResistRefChannelsData[] data);

        
    public enum BrainBit2ChannelMode : byte
    {
        ChModeShort,
        ChModeNormal
    };
    
    public struct BrainBit2AmplifierParam
    {
        public BrainBit2ChannelMode[] ChSignalMode;
        public bool[] ChResistUse;
        public SensorGain[] ChGain;
        public GenCurrent Current;
    }

    
    #endregion

    public delegate void SensorsChanged(IScanner scanner, IReadOnlyList<SensorInfo> sensors);

    public delegate void BatteryChanged(ISensor sensor, int battPower);
    
    public delegate void SensorStateChanged(ISensor sensor, SensorState sensorState);
    
    
}
