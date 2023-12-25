#if UNITY_IOS
#define __IOS__
#endif

using System;
using System.Runtime.InteropServices;
using System.Text;

namespace NeuroSDK
{
    internal class SdkLib
    {
        public const string LibName = "neurosdk2";
#if __IOS__
        public const string LibNameIOS = "__Internal";
#else
        public const string LibNameIOS = LibName;
#endif
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct CallibriSignalDataNative
    {
        public uint PackNum;
        public IntPtr Samples;
        public uint SzSamples;
    };
    [StructLayout(LayoutKind.Sequential)]
    internal struct CallibriRespirationDataNative
    {
        public uint PackNum;
        public IntPtr Samples;
        public uint SzSamples;
    }
    [StructLayout(LayoutKind.Sequential)]
    public struct SignalChannelsDataNative
    {
        public uint PackNum;
        public byte Marker;
        public uint SzSamples;
        public IntPtr Samples;
    }
    [StructLayout(LayoutKind.Sequential)]
    public struct ResistChannelsDataNative
    {
        public uint PackNum;
        public double A1;
        public double A2;
        public double Bias;
        public uint SzValues;
        public IntPtr Values;
    }
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorFileDataNative
    {
        public uint OffsetStart;
	    public uint DataAmount;
	    public uint SzData;
	    public IntPtr Data;
    }
    [StructLayout(LayoutKind.Sequential)]
    internal struct CallibriNextChannelDataNative
    {
        public uint PackNum;
        public uint SzSamples;
        public IntPtr Samples;
    };
    [StructLayout(LayoutKind.Sequential)]
    public struct NeuroBAMResistChannelsDataNative
    {
        public uint PackNum;
        public double Fp1;
        public double Fp2;
        public double Bias;
        public uint SzValues;
        public IntPtr Values;
    }
    [StructLayout(LayoutKind.Sequential)]
    public struct ResistRefChannelsDataNative
    {
        public uint PackNum;
        public uint SzSamples;
        public uint SzReferents;
        public IntPtr Samples;
        public IntPtr Referents;
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    internal delegate void SensorsCallbackScanner(IntPtr ptr, IntPtr sensors, IntPtr szSensors, IntPtr userData);


    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void BatteryCallbackSensor(IntPtr ptr, int battPower, IntPtr userData);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void MEMSDataCallbackSensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] MEMSData[] dataArray, [In] IntPtr dataSize, IntPtr userData);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void ConnectionStateCallbackSensor(IntPtr ptr, SensorState state, IntPtr userData);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void ElectrodeStateCallbackCallibriSensor(IntPtr ptr, CallibriElectrodeState state, IntPtr userData);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void QuaternionDataCallbackSensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] QuaternionData[] dataArray, [In] IntPtr dataSize, IntPtr userData);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void SignalCallbackCallibriSensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] CallibriSignalDataNative[] dataArray, [In] int dataSize, IntPtr userData);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void RespirationCallbackCallibriSensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] CallibriRespirationDataNative[] dataArray, [In] int dataSize, IntPtr userData);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void EnvelopeDataCallbackCallibriSensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] CallibriEnvelopeData[] dataArray, [In] int dataSize, IntPtr userData);


    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void AmpModeCallbackSensor(IntPtr ptr, SensorAmpMode mode, IntPtr userData);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void FPGDataCallbackSensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] FPGData[] dataArray, [In] int dataSize, IntPtr userData);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void ResistCallbackBrainBitSensor(IntPtr ptr, BrainBitResistData data, IntPtr userData);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void SignalDataCallbackBrainBitSensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] BrainBitSignalData[] dataArray, [In] int dataSize, IntPtr userData);








 
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void SignalCallbackBrainBit2Sensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] SignalChannelsDataNative[] dataArray, [In] int dataSize, IntPtr userData);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate void ResistCallbackBrainBit2Sensor(IntPtr ptr, [In, MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 2)] ResistRefChannelsDataNative[] dataArray, [In] int dataSize, IntPtr userData);


    internal interface ISDKApi
    {
        IntPtr CreateScanner(SensorFamily[] filters, out OpStatus outStatus);
        void FreeScanner(IntPtr ptr);
        byte StartScanner(IntPtr ptr, out OpStatus outStatus);
        byte StopScanner(IntPtr ptr, out OpStatus outStatus);
        byte SensorsScanner(IntPtr ptr, out SensorInfo[] sensors, out OpStatus outStatus);
        byte AddSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveSensorsCallbackScanner(IntPtr handle);
        IntPtr CreateSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus);
        void FreeSensor(IntPtr ptr);

        byte ConnectSensor(IntPtr ptr, out OpStatus outStatus);
        byte DisconnectSensor(IntPtr ptr, out OpStatus outStatus);

        int GetFeaturesCountSensor(IntPtr ptr);
        byte GetFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus);
        sbyte IsSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature);

        int GetCommandsCountSensor(IntPtr ptr);
        byte GetCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus);
        sbyte IsSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand);

        int GetParametersCountSensor(IntPtr ptr);
        byte GetParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus);
        sbyte IsSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter);

        byte ExecCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus);
        SensorFamily GetFamilySensor(IntPtr ptr);

        byte ReadNameSensor(IntPtr ptr, out string nameOut, out OpStatus outStatus);
        byte WriteNameSensor(IntPtr ptr, string name, out OpStatus outStatus);
        byte ReadStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus);
        byte ReadAddressSensor(IntPtr ptr, out string addressOut, out OpStatus outStatus);
        byte ReadSerialNumberSensor(IntPtr ptr, out string serialNumberOut, out OpStatus outStatus);
        byte WriteSerialNumberSensor(IntPtr ptr, string serialNumber, out OpStatus outStatus);
        byte ReadBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus);

        byte ReadSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
        byte ReadGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus);
        byte ReadDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus);
        byte ReadSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        byte ReadAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus);
        byte WriteAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus);
        byte ReadGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus);
        byte WriteGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus);

        byte AddMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveMEMSDataCallback(IntPtr handle);

        byte ReadFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus);
        
        byte ReadVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus);
        
    	byte ReadSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
	    byte ReadSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
	    byte ReadIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus);
	    byte WriteIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus);
	    byte ReadRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus);
	    byte WriteRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus);
	    byte ReadAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus);

	    byte AddAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveAmpModeCallback(IntPtr handle);
    	byte PingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus);
    	byte AddFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveFPGDataCallback(IntPtr handle);
	    byte WriteSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus);


        byte AddBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveBatteryCallback(IntPtr handle);

        byte AddConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveConnectionStateCallback(IntPtr handle);

        byte ReadHardwareFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus);
	    byte WriteHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, out OpStatus outStatus);
        byte WriteGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus);
        byte ReadExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus);
	    byte WriteExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus);
        byte ReadColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus);
        byte GetSupportedFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus);
        byte IsSupportedFilterSensor(IntPtr ptr, SensorFilter filter);
        byte ReadElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus);
        byte ReadSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        void ReadColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut);

		byte WriteFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus);
	    byte WriteDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus);
	    byte ReadADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus);
	    byte WriteADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus);

	    byte ReadStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus);
	    byte ReadStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus);
	    byte WriteStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus);
	    byte ReadMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus);
	    byte WriteMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus);
	    byte ReadMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus);
	    byte WriteMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus);
	    byte ReadMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus);
	    byte ReadMEMSCalibrateStateCallibri(IntPtr ptr, out bool state, out OpStatus outStatus);
	    byte ReadSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
	    byte GetSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus);
	    byte SetSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus);

	    byte AddSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveSignalCallbackCallibri(IntPtr handle);
	    byte AddRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveRespirationCallbackCallibri(IntPtr handle);
	    byte AddElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveElectrodeStateCallbackCallibri(IntPtr handle);
	    byte AddEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveEnvelopeDataCallbackCallibri(IntPtr handle);
        
	    byte AddQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveQuaternionDataCallback(IntPtr handle);

	    byte AddResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveResistCallbackBrainBit(IntPtr handle);
        byte AddSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveSignalDataCallbackBrainBit(IntPtr handle);









        // -----===== BrainBit2 =====-----
        byte AddSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveSignalCallbackBrainBit2(IntPtr handle);
        byte AddResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        void RemoveResistCallbackBrainBit2(IntPtr handle);

        byte ReadSupportedChannelsBrainBit2(IntPtr ptr, out EEGChannelInfo[] channelsOut, out OpStatus outStatus);

	    byte ReadAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus);
	    byte WriteAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus);

    }
#if !__IOS__
    internal sealed class SDKApiX32 : ISDKApi
    {
        public const string LibNameOS = SdkLib.LibName + "-x32";
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createScanner(SensorFamily[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeScanner(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte startScanner(IntPtr ptr, out OpStatus outStatus, int numOfTrying);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte stopScanner(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte sensorsScanner(IntPtr ptr, [In, Out] SensorInfo[] sensors, ref int szSensorsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSensorsCallbackScanner(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte connectSensor(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte disconnectSensor(IntPtr ptr, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getFeaturesCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getCommandsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getParametersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getChannelsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte execCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern SensorFamily getFamilySensor(IntPtr ptr);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readNameSensor(IntPtr ptr, StringBuilder nameOut, int szNameIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeNameSensor(IntPtr ptr, StringBuilder name, int szName, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAddressSensor(IntPtr ptr, StringBuilder addressOut, int szAddressIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSerialNumberSensor(IntPtr ptr, StringBuilder serialNumberOut, int szSerialNumberIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSerialNumberSensor(IntPtr ptr, StringBuilder serialNumber, int szSerialNumber, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus);
        
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus);
                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeMEMSDataCallback(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus);
        
        		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  readAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus);

    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  addAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern void removeAmpModeCallback(IntPtr handle);
	[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  pingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  addFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeFPGDataCallback(IntPtr handle);
		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus);


        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeBatteryCallback(IntPtr handle);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeConnectionStateCallback(IntPtr handle);

                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readHardwareFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getSupportedFiltersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSupportedFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte isSupportedFilterSensor(IntPtr ptr, SensorFilter filter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void readColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte setSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMEMSCalibrateStateCallibri(IntPtr ptr, out byte state, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeRespirationCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeElectrodeStateCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeEnvelopeDataCallbackCallibri(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeQuaternionDataCallback(IntPtr handle);
	    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalDataCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSupportedChannelsBrainBit2(IntPtr ptr, [In, Out] EEGChannelInfo[] channelsOut, ref int szchannelsInOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus);

        
        public IntPtr CreateScanner(SensorFamily[] filters, out OpStatus outStatus)
        {
            return createScanner(filters, filters.Length, out outStatus);
        }
        public void FreeScanner(IntPtr ptr)
        {
            freeScanner(ptr);
        }
        public byte StartScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return startScanner(ptr, out outStatus, 1);
        }
        public byte StopScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return stopScanner(ptr, out outStatus);
        }
        public byte SensorsScanner(IntPtr ptr, out SensorInfo[] sensors, out OpStatus outStatus)
        {
            int sz = 64;
            SensorInfo[] sensorsArr = new SensorInfo[sz];
            var res = sensorsScanner(ptr, sensorsArr, ref sz, out outStatus);
            sensors = new SensorInfo[sz];
            Array.Copy(sensorsArr, 0, sensors, 0, sz);
            return res;
        }
        public byte AddSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSensorsCallbackScanner(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSensorsCallbackScanner(IntPtr handle)
        {
            removeSensorsCallbackScanner(handle);
        }
        public IntPtr CreateSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus)
        {
            return createSensor(ptr, sensor, out outStatus);
        }
        public void FreeSensor(IntPtr ptr)
        {
            freeSensor(ptr);
        }
        public byte ConnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return connectSensor(ptr, out outStatus);
        }
        public byte DisconnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return disconnectSensor(ptr, out outStatus);
        }

        public int GetFeaturesCountSensor(IntPtr ptr)
        {
            return getFeaturesCountSensor(ptr);
        }
        public byte GetFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus)
        {
            return getFeaturesSensor(ptr, sensorFeatures, ref szSensorFeaturesInOut, out outStatus);
        }
        public sbyte IsSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature)
        {
            return isSupportedFeatureSensor(ptr, sensorFeature);
        }

        public int GetCommandsCountSensor(IntPtr ptr)
        {
            return getCommandsCountSensor(ptr);
        }
        public byte GetCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus)
        {
            return getCommandsSensor(ptr, sensorCommands, ref szSensorCommandsInOut, out outStatus);
        }
        public sbyte IsSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand)
        {
            return isSupportedCommandSensor(ptr, sensorCommand);
        }

        public int GetParametersCountSensor(IntPtr ptr)
        {
            return getParametersCountSensor(ptr);
        }
        public byte GetParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus)
        {
            return getParametersSensor(ptr, sensorParameters, ref szSensorParametersInOut, out outStatus);
        }
        public sbyte IsSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter)
        {
            return isSupportedParameterSensor(ptr, sensorParameter);
        }

        public byte ExecCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus)
        {
            return execCommandSensor(ptr, sensorCommand, out outStatus);
        }
        public SensorFamily GetFamilySensor(IntPtr ptr)
        {
            return getFamilySensor(ptr);
        }

        public byte ReadNameSensor(IntPtr ptr, out string nameOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorNameLen);
            var res = readNameSensor(ptr, sb, sb.Capacity, out outStatus);
            nameOut = sb.ToString();
            return res;
        }
        public byte WriteNameSensor(IntPtr ptr, string name, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(name);
            return writeNameSensor(ptr, sb, sb.Capacity, out outStatus);
        }

        public byte ReadStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus)
        {
            return readStateSensor(ptr, out stateOut, out outStatus);
        }
        public byte ReadAddressSensor(IntPtr ptr, out string addressOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorAdrLen);
            var res = readAddressSensor(ptr, sb, sb.Capacity, out outStatus);
            addressOut = sb.ToString();
            return res;
        }
        public byte ReadSerialNumberSensor(IntPtr ptr, out string serialNumberOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorSNLen);
            var res = readSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
            serialNumberOut = sb.ToString();
            return res;
        }
        public byte WriteSerialNumberSensor(IntPtr ptr, string serialNumber, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(serialNumber);
            return writeSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
        }
        public byte ReadBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus)
        {
            return readBattPowerSensor(ptr, out battPowerOut, out outStatus);
        }
        
        public byte ReadSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencySensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        
        public byte ReadGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus)
        {
            return readGainSensor(ptr, out gainOut, out outStatus);
        }
        
        public byte ReadDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus)
        {
            return readDataOffsetSensor(ptr, out dataOffsetOut, out outStatus);
        }
                public byte ReadSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyMEMSSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus)
        {
            return readAccelerometerSensSensor(ptr, out accSensOut, out outStatus);
        }
        public byte WriteAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus)
        {
            return writeAccelerometerSensSensor(ptr, accSens, out outStatus);
        }
        public byte ReadGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus)
        {
            return readGyroscopeSensSensor(ptr, out gyroSensOut, out outStatus);
        }
        public byte WriteGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus)
        {
            return writeGyroscopeSensSensor(ptr, gyroSens, out outStatus);
        }
        public byte AddMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addMEMSDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveMEMSDataCallback(IntPtr handle)
        {
            removeMEMSDataCallback(handle);
        }

        public byte ReadFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus)
        {
            return readFirmwareModeSensor(ptr, out modeOut, out outStatus);
        }
        
        public byte ReadVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus)
        {
            return readVersionSensor(ptr, out versionOut, out outStatus);
        }
        	    public byte ReadSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyResistSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadHardwareFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = 64;
            SensorFilter[] sensorsArr = new SensorFilter[sz];
            var res = readHardwareFiltersSensor(ptr, sensorsArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[sz];
            Array.Copy(sensorsArr, 0, filtersOut, 0, sz);
            return res;
        }
        public byte WriteHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, out OpStatus outStatus)
        {
            return writeHardwareFiltersSensor(ptr, filters, filters.Length, out outStatus);
        }
        public byte WriteGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus)
        {
            return writeGainSensor(ptr, gain, out outStatus);
        }
        public byte ReadExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus)
        {
            return readExternalSwitchSensor(ptr, out extSwInputOut, out outStatus);
        }
        public byte WriteExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus)
        {
            return writeExternalSwitchSensor(ptr, extSwInput, out outStatus);
        }
        public byte ReadColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus)
        {
            return readColorCallibri(ptr, out callibriColorOut, out outStatus);
        }
        public byte GetSupportedFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = getSupportedFiltersCountSensor(ptr);
            SensorFilter[] filtersArr = new SensorFilter[sz];
            var res = getSupportedFiltersSensor(ptr, filtersArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[outStatus.Success ? sz : 0];
            if (outStatus.Success)
            {
                Array.Copy(filtersArr, 0, filtersOut, 0, sz);
            }
            return res;
        }
        public byte IsSupportedFilterSensor(IntPtr ptr, SensorFilter filter)
        {
            return isSupportedFilterSensor(ptr, filter);
        }
        public byte ReadElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus)
        {
            return readElectrodeStateCallibri(ptr, out electrodeStateOut, out outStatus);
        }
        public byte ReadSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyEnvelopeSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public void ReadColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut)
        {
            readColorInfo(sensorInfo, out callibriColorOut);
        }
        public byte WriteFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus)
        {
            return writeFirmwareModeSensor(ptr, mode, out outStatus);
        }
        public byte WriteDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus)
        {
            return writeDataOffsetSensor(ptr, dataOffset, out outStatus);
        }
        public byte ReadADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus)
        {
            return readADCInputSensor(ptr, out adcInputOut, out outStatus);
        }
        public byte WriteADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus)
        {
            return writeADCInputSensor(ptr, adcInput, out outStatus);
        }

        public byte ReadStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus)
        {
            return readStimulatorAndMAStateCallibri(ptr, out stimulatorMAStateOut, out outStatus);
        }
        public byte ReadStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus)
        {
            return readStimulatorParamCallibri(ptr, out stimulationParamsOut, out outStatus);
        }
        public byte WriteStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus)
        {
            return writeStimulatorParamCallibri(ptr, stimulationParams, out outStatus);
        }
        public byte ReadMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus)
        {
            return readMotionAssistantParamCallibri(ptr, out motionAssistantParamsOut, out outStatus);
        }
        public byte WriteMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus)
        {
            return writeMotionAssistantParamCallibri(ptr, motionAssistantParams, out outStatus);
        }
        public byte ReadMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus)
        {
            return readMotionCounterParamCallibri(ptr, out motionCounterParamOut, out outStatus);
        }
        public byte WriteMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus)
        {
            return writeMotionCounterParamCallibri(ptr, motionCounterParam, out outStatus);
        }
        public byte ReadMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus)
        {
            return readMotionCounterCallibri(ptr, out motionCounterOut, out outStatus);
        }

        public byte GetSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus)
        {
            return getSignalSettingsCallibri(ptr, out callibriSignalTypeOut, out outStatus);
        }
        public byte SetSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus)
        {
            return setSignalSettingsCallibri(ptr, callibriSignalType, out outStatus);
        }
        public byte ReadMEMSCalibrateStateCallibri(IntPtr ptr, out bool state, out OpStatus outStatus)
        {
            byte bState;
            var res = readMEMSCalibrateStateCallibri(ptr, out bState, out outStatus);
            state = bState != 0;
            return res;
        }
        public byte ReadSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyRespSensor(ptr, out samplingFrequencyOut, out outStatus);
        }

        public byte AddSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackCallibri(IntPtr handle)
        {
            removeSignalCallbackCallibri(handle);
        }
        public byte AddRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addRespirationCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveRespirationCallbackCallibri(IntPtr handle)
        {
            removeRespirationCallbackCallibri(handle);
        }
        public byte AddElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addElectrodeStateCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveElectrodeStateCallbackCallibri(IntPtr handle)
        {
            removeElectrodeStateCallbackCallibri(handle);
        }
        public byte AddEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addEnvelopeDataCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveEnvelopeDataCallbackCallibri(IntPtr handle)
        {
            removeEnvelopeDataCallbackCallibri(handle);
        }

        public byte AddQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addQuaternionDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveQuaternionDataCallback(IntPtr handle)
        {
            removeQuaternionDataCallback(handle);
        }
        public byte AddResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit(IntPtr handle)
        {
            removeResistCallbackBrainBit(handle);
        }
        public byte AddSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalDataCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalDataCallbackBrainBit(IntPtr handle)
        {
            removeSignalDataCallbackBrainBit(handle);
        }
        public byte ReadSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyFPGSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readIrAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus)
        {
            return writeIrAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readRedAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus)
        {
            return writeRedAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus)
        {
            return readAmpMode(ptr, out modeOut, out outStatus);
        }

        public byte AddAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addAmpModeCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveAmpModeCallback(IntPtr handle)
        {
            removeAmpModeCallback(handle);
        }
	    public byte PingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus)
        {
            return pingNeuroSmart(ptr, marker, out outStatus);
        }
        public byte AddFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addFPGDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveFPGDataCallback(IntPtr handle)
        {
            removeFPGDataCallback(handle);
        }
	    public byte WriteSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus)
        {
            return writeSamplingFrequencySensor(ptr, samplingFrequency, out outStatus);
        }


        public byte AddBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addBatteryCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveBatteryCallback(IntPtr handle)
        {
            removeBatteryCallback(handle);
        }
        
        public byte AddConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addConnectionStateCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveConnectionStateCallback(IntPtr handle)
        {
            removeConnectionStateCallback(handle);
        }
                public byte AddSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackBrainBit2(IntPtr handle)
        {
            removeSignalCallbackBrainBit2(handle);
        }
        public byte AddResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit2(IntPtr handle)
        {
            removeResistCallbackBrainBit2(handle);
        }

        public byte ReadSupportedChannelsBrainBit2(IntPtr ptr, out EEGChannelInfo[] channelsOut, out OpStatus outStatus)
        {
            var cnt = getChannelsCountSensor(ptr);
            channelsOut = new EEGChannelInfo[cnt];
            var res = readSupportedChannelsBrainBit2(ptr, channelsOut, ref cnt, out outStatus);
            if (cnt != channelsOut.Length)
            {
                var chs = new EEGChannelInfo[cnt];
                Array.Copy(channelsOut, chs, cnt);
                channelsOut = chs;
            }
            return res;
        }
        public byte ReadAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus)
        {
            var res = readAmplifierParamBrainBit2(ptr, out ampParamOut, out outStatus);
            if (outStatus.Success)
            {
                var cnt = getChannelsCountSensor(ptr);
                if (ampParamOut.ChSignalMode.Length != cnt)
                {
                    var chSignalMode = new BrainBit2ChannelMode[cnt];
                    Array.Copy(ampParamOut.ChSignalMode, chSignalMode, Math.Min(cnt, ampParamOut.ChSignalMode.Length));
                    ampParamOut.ChSignalMode = chSignalMode;
                }
                if (ampParamOut.ChResistUse.Length != cnt)
                {
                    var chResistUse = new bool[cnt];
                    Array.Copy(ampParamOut.ChResistUse, chResistUse, Math.Min(cnt, ampParamOut.ChResistUse.Length));
                    ampParamOut.ChResistUse = chResistUse;
                }
                if (ampParamOut.ChGain.Length != cnt)
                {
                    var chGain = new SensorGain[cnt];
                    Array.Copy(ampParamOut.ChGain, chGain, Math.Min(cnt, ampParamOut.ChGain.Length));
                    ampParamOut.ChGain = chGain;
                }
            }
            return res;
        }
        public byte WriteAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus)
        {
            var cnt = SdkLibConst.BrainBit2MaxChCount;
            if (ampParam.ChSignalMode.Length != cnt)
            {
                var chSignalMode = new BrainBit2ChannelMode[cnt];
                Array.Copy(ampParam.ChSignalMode, chSignalMode, Math.Min(cnt, ampParam.ChSignalMode.Length));
                ampParam.ChSignalMode = chSignalMode;
            }
            if (ampParam.ChResistUse.Length != cnt)
            {
                var chResistUse = new bool[cnt];
                Array.Copy(ampParam.ChResistUse, chResistUse, Math.Min(cnt, ampParam.ChResistUse.Length));
                ampParam.ChResistUse = chResistUse;
            }
            if (ampParam.ChGain.Length != cnt)
            {
                var chGain = new SensorGain[cnt];
                Array.Copy(ampParam.ChGain, chGain, Math.Min(cnt, ampParam.ChGain.Length));
                ampParam.ChGain = chGain;
            }
            return writeAmplifierParamBrainBit2(ptr, ampParam, out outStatus);
        }

    }
    internal sealed class SDKApiX64 : ISDKApi
    {
        public const string LibNameOS = SdkLib.LibName + "-x64";
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createScanner(SensorFamily[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeScanner(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte startScanner(IntPtr ptr, out OpStatus outStatus, int numOfTrying);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte stopScanner(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte sensorsScanner(IntPtr ptr, [In, Out] SensorInfo[] sensors, ref int szSensorsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSensorsCallbackScanner(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte connectSensor(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte disconnectSensor(IntPtr ptr, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getFeaturesCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getCommandsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getParametersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getChannelsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte execCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern SensorFamily getFamilySensor(IntPtr ptr);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readNameSensor(IntPtr ptr, StringBuilder nameOut, int szNameIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeNameSensor(IntPtr ptr, StringBuilder name, int szName, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAddressSensor(IntPtr ptr, StringBuilder addressOut, int szAddressIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSerialNumberSensor(IntPtr ptr, StringBuilder serialNumberOut, int szSerialNumberIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSerialNumberSensor(IntPtr ptr, StringBuilder serialNumber, int szSerialNumber, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus);
        
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus);
                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeMEMSDataCallback(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus);
        
        		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  readAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus);

    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  addAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern void removeAmpModeCallback(IntPtr handle);
	[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  pingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  addFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeFPGDataCallback(IntPtr handle);
		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus);


        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeBatteryCallback(IntPtr handle);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeConnectionStateCallback(IntPtr handle);

                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readHardwareFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getSupportedFiltersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSupportedFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte isSupportedFilterSensor(IntPtr ptr, SensorFilter filter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void readColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte setSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMEMSCalibrateStateCallibri(IntPtr ptr, out byte state, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeRespirationCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeElectrodeStateCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeEnvelopeDataCallbackCallibri(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeQuaternionDataCallback(IntPtr handle);
	    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalDataCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSupportedChannelsBrainBit2(IntPtr ptr, [In, Out] EEGChannelInfo[] channelsOut, ref int szchannelsInOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus);


        public IntPtr CreateScanner(SensorFamily[] filters, out OpStatus outStatus)
        {
            return createScanner(filters, filters.Length, out outStatus);
        }
        public void FreeScanner(IntPtr ptr)
        {
            freeScanner(ptr);
        }
        public byte StartScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return startScanner(ptr, out outStatus, 1);
        }
        public byte StopScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return stopScanner(ptr, out outStatus);
        }
        public byte SensorsScanner(IntPtr ptr, out SensorInfo[] sensors, out OpStatus outStatus)
        {
            int sz = 64;
            SensorInfo[] sensorsArr = new SensorInfo[sz];
            var res = sensorsScanner(ptr, sensorsArr, ref sz, out outStatus);
            sensors = new SensorInfo[sz];
            Array.Copy(sensorsArr, 0, sensors, 0, sz);
            return res;
        }
        public byte AddSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSensorsCallbackScanner(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSensorsCallbackScanner(IntPtr handle)
        {
            removeSensorsCallbackScanner(handle);
        }
        public IntPtr CreateSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus)
        {
            return createSensor(ptr, sensor, out outStatus);
        }
        public void FreeSensor(IntPtr ptr)
        {
            freeSensor(ptr);
        }
        public byte ConnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return connectSensor(ptr, out outStatus);
        }
        public byte DisconnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return disconnectSensor(ptr, out outStatus);
        }

        public int GetFeaturesCountSensor(IntPtr ptr)
        {
            return getFeaturesCountSensor(ptr);
        }
        public byte GetFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus)
        {
            return getFeaturesSensor(ptr, sensorFeatures, ref szSensorFeaturesInOut, out outStatus);
        }
        public sbyte IsSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature)
        {
            return isSupportedFeatureSensor(ptr, sensorFeature);
        }

        public int GetCommandsCountSensor(IntPtr ptr)
        {
            return getCommandsCountSensor(ptr);
        }
        public byte GetCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus)
        {
            return getCommandsSensor(ptr, sensorCommands, ref szSensorCommandsInOut, out outStatus);
        }
        public sbyte IsSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand)
        {
            return isSupportedCommandSensor(ptr, sensorCommand);
        }

        public int GetParametersCountSensor(IntPtr ptr)
        {
            return getParametersCountSensor(ptr);
        }
        public byte GetParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus)
        {
            return getParametersSensor(ptr, sensorParameters, ref szSensorParametersInOut, out outStatus);
        }
        public sbyte IsSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter)
        {
            return isSupportedParameterSensor(ptr, sensorParameter);
        }

        public byte ExecCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus)
        {
            return execCommandSensor(ptr, sensorCommand, out outStatus);
        }
        public SensorFamily GetFamilySensor(IntPtr ptr)
        {
            return getFamilySensor(ptr);
        }

        public byte ReadNameSensor(IntPtr ptr, out string nameOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorNameLen);
            var res = readNameSensor(ptr, sb, sb.Capacity, out outStatus);
            nameOut = sb.ToString();
            return res;
        }
        public byte WriteNameSensor(IntPtr ptr, string name, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(name);
            return writeNameSensor(ptr, sb, sb.Capacity, out outStatus);
        }

        public byte ReadStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus)
        {
            return readStateSensor(ptr, out stateOut, out outStatus);
        }
        public byte ReadAddressSensor(IntPtr ptr, out string addressOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorAdrLen);
            var res = readAddressSensor(ptr, sb, sb.Capacity, out outStatus);
            addressOut = sb.ToString();
            return res;
        }
        public byte ReadSerialNumberSensor(IntPtr ptr, out string serialNumberOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorSNLen);
            var res = readSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
            serialNumberOut = sb.ToString();
            return res;
        }
        public byte WriteSerialNumberSensor(IntPtr ptr, string serialNumber, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(serialNumber);
            return writeSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
        }
        public byte ReadBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus)
        {
            return readBattPowerSensor(ptr, out battPowerOut, out outStatus);
        }
        
        public byte ReadSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencySensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        
        public byte ReadGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus)
        {
            return readGainSensor(ptr, out gainOut, out outStatus);
        }
        
        public byte ReadDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus)
        {
            return readDataOffsetSensor(ptr, out dataOffsetOut, out outStatus);
        }
                public byte ReadSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyMEMSSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus)
        {
            return readAccelerometerSensSensor(ptr, out accSensOut, out outStatus);
        }
        public byte WriteAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus)
        {
            return writeAccelerometerSensSensor(ptr, accSens, out outStatus);
        }
        public byte ReadGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus)
        {
            return readGyroscopeSensSensor(ptr, out gyroSensOut, out outStatus);
        }
        public byte WriteGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus)
        {
            return writeGyroscopeSensSensor(ptr, gyroSens, out outStatus);
        }
        public byte AddMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addMEMSDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveMEMSDataCallback(IntPtr handle)
        {
            removeMEMSDataCallback(handle);
        }

        public byte ReadFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus)
        {
            return readFirmwareModeSensor(ptr, out modeOut, out outStatus);
        }
        
        public byte ReadVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus)
        {
            return readVersionSensor(ptr, out versionOut, out outStatus);
        }
        	    public byte ReadSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyResistSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadHardwareFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = 64;
            SensorFilter[] sensorsArr = new SensorFilter[sz];
            var res = readHardwareFiltersSensor(ptr, sensorsArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[sz];
            Array.Copy(sensorsArr, 0, filtersOut, 0, sz);
            return res;
        }
        public byte WriteHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, out OpStatus outStatus)
        {
            return writeHardwareFiltersSensor(ptr, filters, filters.Length, out outStatus);
        }
        public byte WriteGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus)
        {
            return writeGainSensor(ptr, gain, out outStatus);
        }
        public byte ReadExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus)
        {
            return readExternalSwitchSensor(ptr, out extSwInputOut, out outStatus);
        }
        public byte WriteExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus)
        {
            return writeExternalSwitchSensor(ptr, extSwInput, out outStatus);
        }
        public byte ReadColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus)
        {
            return readColorCallibri(ptr, out callibriColorOut, out outStatus);
        }
        public byte GetSupportedFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = getSupportedFiltersCountSensor(ptr);
            SensorFilter[] filtersArr = new SensorFilter[sz];
            var res = getSupportedFiltersSensor(ptr, filtersArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[outStatus.Success ? sz : 0];
            if (outStatus.Success)
            {
                Array.Copy(filtersArr, 0, filtersOut, 0, sz);
            }
            return res;
        }
        public byte IsSupportedFilterSensor(IntPtr ptr, SensorFilter filter)
        {
            return isSupportedFilterSensor(ptr, filter);
        }
        public byte ReadElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus)
        {
            return readElectrodeStateCallibri(ptr, out electrodeStateOut, out outStatus);
        }
        public byte ReadSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyEnvelopeSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public void ReadColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut)
        {
            readColorInfo(sensorInfo, out callibriColorOut);
        }
        public byte WriteFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus)
        {
            return writeFirmwareModeSensor(ptr, mode, out outStatus);
        }
        public byte WriteDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus)
        {
            return writeDataOffsetSensor(ptr, dataOffset, out outStatus);
        }
        public byte ReadADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus)
        {
            return readADCInputSensor(ptr, out adcInputOut, out outStatus);
        }
        public byte WriteADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus)
        {
            return writeADCInputSensor(ptr, adcInput, out outStatus);
        }

        public byte ReadStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus)
        {
            return readStimulatorAndMAStateCallibri(ptr, out stimulatorMAStateOut, out outStatus);
        }
        public byte ReadStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus)
        {
            return readStimulatorParamCallibri(ptr, out stimulationParamsOut, out outStatus);
        }
        public byte WriteStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus)
        {
            return writeStimulatorParamCallibri(ptr, stimulationParams, out outStatus);
        }
        public byte ReadMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus)
        {
            return readMotionAssistantParamCallibri(ptr, out motionAssistantParamsOut, out outStatus);
        }
        public byte WriteMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus)
        {
            return writeMotionAssistantParamCallibri(ptr, motionAssistantParams, out outStatus);
        }
        public byte ReadMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus)
        {
            return readMotionCounterParamCallibri(ptr, out motionCounterParamOut, out outStatus);
        }
        public byte WriteMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus)
        {
            return writeMotionCounterParamCallibri(ptr, motionCounterParam, out outStatus);
        }
        public byte ReadMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus)
        {
            return readMotionCounterCallibri(ptr, out motionCounterOut, out outStatus);
        }

        public byte GetSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus)
        {
            return getSignalSettingsCallibri(ptr, out callibriSignalTypeOut, out outStatus);
        }
        public byte SetSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus)
        {
            return setSignalSettingsCallibri(ptr, callibriSignalType, out outStatus);
        }
        public byte ReadMEMSCalibrateStateCallibri(IntPtr ptr, out bool state, out OpStatus outStatus)
        {
            byte bState;
            var res = readMEMSCalibrateStateCallibri(ptr, out bState, out outStatus);
            state = bState != 0;
            return res;
        }
        public byte ReadSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyRespSensor(ptr, out samplingFrequencyOut, out outStatus);
        }

        public byte AddSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackCallibri(IntPtr handle)
        {
            removeSignalCallbackCallibri(handle);
        }
        public byte AddRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addRespirationCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveRespirationCallbackCallibri(IntPtr handle)
        {
            removeRespirationCallbackCallibri(handle);
        }
        public byte AddElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addElectrodeStateCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveElectrodeStateCallbackCallibri(IntPtr handle)
        {
            removeElectrodeStateCallbackCallibri(handle);
        }
        public byte AddEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addEnvelopeDataCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveEnvelopeDataCallbackCallibri(IntPtr handle)
        {
            removeEnvelopeDataCallbackCallibri(handle);
        }

        public byte AddQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addQuaternionDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveQuaternionDataCallback(IntPtr handle)
        {
            removeQuaternionDataCallback(handle);
        }
        public byte AddResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit(IntPtr handle)
        {
            removeResistCallbackBrainBit(handle);
        }
        public byte AddSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalDataCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalDataCallbackBrainBit(IntPtr handle)
        {
            removeSignalDataCallbackBrainBit(handle);
        }
        public byte ReadSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyFPGSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readIrAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus)
        {
            return writeIrAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readRedAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus)
        {
            return writeRedAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus)
        {
            return readAmpMode(ptr, out modeOut, out outStatus);
        }

        public byte AddAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addAmpModeCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveAmpModeCallback(IntPtr handle)
        {
            removeAmpModeCallback(handle);
        }
	    public byte PingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus)
        {
            return pingNeuroSmart(ptr, marker, out outStatus);
        }
        public byte AddFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addFPGDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveFPGDataCallback(IntPtr handle)
        {
            removeFPGDataCallback(handle);
        }
	    public byte WriteSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus)
        {
            return writeSamplingFrequencySensor(ptr, samplingFrequency, out outStatus);
        }


        public byte AddBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addBatteryCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveBatteryCallback(IntPtr handle)
        {
            removeBatteryCallback(handle);
        }
        
        public byte AddConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addConnectionStateCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveConnectionStateCallback(IntPtr handle)
        {
            removeConnectionStateCallback(handle);
        }
                public byte AddSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackBrainBit2(IntPtr handle)
        {
            removeSignalCallbackBrainBit2(handle);
        }
        public byte AddResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit2(IntPtr handle)
        {
            removeResistCallbackBrainBit2(handle);
        }

        public byte ReadSupportedChannelsBrainBit2(IntPtr ptr, out EEGChannelInfo[] channelsOut, out OpStatus outStatus)
        {
            var cnt = getChannelsCountSensor(ptr);
            channelsOut = new EEGChannelInfo[cnt];
            var res = readSupportedChannelsBrainBit2(ptr, channelsOut, ref cnt, out outStatus);
            if (cnt != channelsOut.Length)
            {
                var chs = new EEGChannelInfo[cnt];
                Array.Copy(channelsOut, chs, cnt);
                channelsOut = chs;
            }
            return res;
        }
        public byte ReadAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus)
        {
            var res = readAmplifierParamBrainBit2(ptr, out ampParamOut, out outStatus);
            if (outStatus.Success)
            {
                var cnt = getChannelsCountSensor(ptr);
                if (ampParamOut.ChSignalMode.Length != cnt)
                {
                    var chSignalMode = new BrainBit2ChannelMode[cnt];
                    Array.Copy(ampParamOut.ChSignalMode, chSignalMode, Math.Min(cnt, ampParamOut.ChSignalMode.Length));
                    ampParamOut.ChSignalMode = chSignalMode;
                }
                if (ampParamOut.ChResistUse.Length != cnt)
                {
                    var chResistUse = new bool[cnt];
                    Array.Copy(ampParamOut.ChResistUse, chResistUse, Math.Min(cnt, ampParamOut.ChResistUse.Length));
                    ampParamOut.ChResistUse = chResistUse;
                }
                if (ampParamOut.ChGain.Length != cnt)
                {
                    var chGain = new SensorGain[cnt];
                    Array.Copy(ampParamOut.ChGain, chGain, Math.Min(cnt, ampParamOut.ChGain.Length));
                    ampParamOut.ChGain = chGain;
                }
            }
            return res;
        }
        public byte WriteAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus)
        {
            var cnt = SdkLibConst.BrainBit2MaxChCount;
            if (ampParam.ChSignalMode.Length != cnt)
            {
                var chSignalMode = new BrainBit2ChannelMode[cnt];
                Array.Copy(ampParam.ChSignalMode, chSignalMode, Math.Min(cnt, ampParam.ChSignalMode.Length));
                ampParam.ChSignalMode = chSignalMode;
            }
            if (ampParam.ChResistUse.Length != cnt)
            {
                var chResistUse = new bool[cnt];
                Array.Copy(ampParam.ChResistUse, chResistUse, Math.Min(cnt, ampParam.ChResistUse.Length));
                ampParam.ChResistUse = chResistUse;
            }
            if (ampParam.ChGain.Length != cnt)
            {
                var chGain = new SensorGain[cnt];
                Array.Copy(ampParam.ChGain, chGain, Math.Min(cnt, ampParam.ChGain.Length));
                ampParam.ChGain = chGain;
            }
            return writeAmplifierParamBrainBit2(ptr, ampParam, out outStatus);
        }

    }
    internal sealed class SDKApiArm : ISDKApi
    {
        public const string LibNameOS = SdkLib.LibName + "-arm";
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createScanner(SensorFamily[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeScanner(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte startScanner(IntPtr ptr, out OpStatus outStatus, int numOfTrying);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte stopScanner(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte sensorsScanner(IntPtr ptr, [In, Out] SensorInfo[] sensors, ref int szSensorsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSensorsCallbackScanner(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte connectSensor(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte disconnectSensor(IntPtr ptr, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getFeaturesCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getCommandsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getParametersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getChannelsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte execCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern SensorFamily getFamilySensor(IntPtr ptr);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readNameSensor(IntPtr ptr, StringBuilder nameOut, int szNameIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeNameSensor(IntPtr ptr, StringBuilder name, int szName, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAddressSensor(IntPtr ptr, StringBuilder addressOut, int szAddressIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSerialNumberSensor(IntPtr ptr, StringBuilder serialNumberOut, int szSerialNumberIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSerialNumberSensor(IntPtr ptr, StringBuilder serialNumber, int szSerialNumber, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus);
        
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus);
                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeMEMSDataCallback(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus);
        
        		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  readAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus);

    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  addAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern void removeAmpModeCallback(IntPtr handle);
	[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  pingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  addFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeFPGDataCallback(IntPtr handle);
		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus);


        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeBatteryCallback(IntPtr handle);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeConnectionStateCallback(IntPtr handle);

                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readHardwareFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getSupportedFiltersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSupportedFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte isSupportedFilterSensor(IntPtr ptr, SensorFilter filter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void readColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte setSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMEMSCalibrateStateCallibri(IntPtr ptr, out byte state, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeRespirationCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeElectrodeStateCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeEnvelopeDataCallbackCallibri(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeQuaternionDataCallback(IntPtr handle);
	    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalDataCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSupportedChannelsBrainBit2(IntPtr ptr, [In, Out] EEGChannelInfo[] channelsOut, ref int szchannelsInOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus);


        public IntPtr CreateScanner(SensorFamily[] filters, out OpStatus outStatus)
        {
            return createScanner(filters, filters.Length, out outStatus);
        }
        public void FreeScanner(IntPtr ptr)
        {
            freeScanner(ptr);
        }
        public byte StartScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return startScanner(ptr, out outStatus, 1);
        }
        public byte StopScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return stopScanner(ptr, out outStatus);
        }
        public byte SensorsScanner(IntPtr ptr, out SensorInfo[] sensors, out OpStatus outStatus)
        {
            int sz = 64;
            SensorInfo[] sensorsArr = new SensorInfo[sz];
            var res = sensorsScanner(ptr, sensorsArr, ref sz, out outStatus);
            sensors = new SensorInfo[sz];
            Array.Copy(sensorsArr, 0, sensors, 0, sz);
            return res;
        }
        public byte AddSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSensorsCallbackScanner(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSensorsCallbackScanner(IntPtr handle)
        {
            removeSensorsCallbackScanner(handle);
        }
        public IntPtr CreateSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus)
        {
            return createSensor(ptr, sensor, out outStatus);
        }
        public void FreeSensor(IntPtr ptr)
        {
            freeSensor(ptr);
        }
        public byte ConnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return connectSensor(ptr, out outStatus);
        }
        public byte DisconnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return disconnectSensor(ptr, out outStatus);
        }

        public int GetFeaturesCountSensor(IntPtr ptr)
        {
            return getFeaturesCountSensor(ptr);
        }
        public byte GetFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus)
        {
            return getFeaturesSensor(ptr, sensorFeatures, ref szSensorFeaturesInOut, out outStatus);
        }
        public sbyte IsSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature)
        {
            return isSupportedFeatureSensor(ptr, sensorFeature);
        }

        public int GetCommandsCountSensor(IntPtr ptr)
        {
            return getCommandsCountSensor(ptr);
        }
        public byte GetCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus)
        {
            return getCommandsSensor(ptr, sensorCommands, ref szSensorCommandsInOut, out outStatus);
        }
        public sbyte IsSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand)
        {
            return isSupportedCommandSensor(ptr, sensorCommand);
        }

        public int GetParametersCountSensor(IntPtr ptr)
        {
            return getParametersCountSensor(ptr);
        }
        public byte GetParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus)
        {
            return getParametersSensor(ptr, sensorParameters, ref szSensorParametersInOut, out outStatus);
        }
        public sbyte IsSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter)
        {
            return isSupportedParameterSensor(ptr, sensorParameter);
        }

        public byte ExecCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus)
        {
            return execCommandSensor(ptr, sensorCommand, out outStatus);
        }
        public SensorFamily GetFamilySensor(IntPtr ptr)
        {
            return getFamilySensor(ptr);
        }

        public byte ReadNameSensor(IntPtr ptr, out string nameOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorNameLen);
            var res = readNameSensor(ptr, sb, sb.Capacity, out outStatus);
            nameOut = sb.ToString();
            return res;
        }
        public byte WriteNameSensor(IntPtr ptr, string name, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(name);
            return writeNameSensor(ptr, sb, sb.Capacity, out outStatus);
        }

        public byte ReadStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus)
        {
            return readStateSensor(ptr, out stateOut, out outStatus);
        }
        public byte ReadAddressSensor(IntPtr ptr, out string addressOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorAdrLen);
            var res = readAddressSensor(ptr, sb, sb.Capacity, out outStatus);
            addressOut = sb.ToString();
            return res;
        }
        public byte ReadSerialNumberSensor(IntPtr ptr, out string serialNumberOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorSNLen);
            var res = readSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
            serialNumberOut = sb.ToString();
            return res;
        }
        public byte WriteSerialNumberSensor(IntPtr ptr, string serialNumber, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(serialNumber);
            return writeSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
        }
        public byte ReadBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus)
        {
            return readBattPowerSensor(ptr, out battPowerOut, out outStatus);
        }
        
        public byte ReadSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencySensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        
        public byte ReadGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus)
        {
            return readGainSensor(ptr, out gainOut, out outStatus);
        }
        
        public byte ReadDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus)
        {
            return readDataOffsetSensor(ptr, out dataOffsetOut, out outStatus);
        }
                public byte ReadSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyMEMSSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus)
        {
            return readAccelerometerSensSensor(ptr, out accSensOut, out outStatus);
        }
        public byte WriteAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus)
        {
            return writeAccelerometerSensSensor(ptr, accSens, out outStatus);
        }
        public byte ReadGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus)
        {
            return readGyroscopeSensSensor(ptr, out gyroSensOut, out outStatus);
        }
        public byte WriteGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus)
        {
            return writeGyroscopeSensSensor(ptr, gyroSens, out outStatus);
        }
        public byte AddMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addMEMSDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveMEMSDataCallback(IntPtr handle)
        {
            removeMEMSDataCallback(handle);
        }

        public byte ReadFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus)
        {
            return readFirmwareModeSensor(ptr, out modeOut, out outStatus);
        }
        
        public byte ReadVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus)
        {
            return readVersionSensor(ptr, out versionOut, out outStatus);
        }
        	    public byte ReadSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyResistSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadHardwareFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = 64;
            SensorFilter[] sensorsArr = new SensorFilter[sz];
            var res = readHardwareFiltersSensor(ptr, sensorsArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[sz];
            Array.Copy(sensorsArr, 0, filtersOut, 0, sz);
            return res;
        }
        public byte WriteHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, out OpStatus outStatus)
        {
            return writeHardwareFiltersSensor(ptr, filters, filters.Length, out outStatus);
        }
        public byte WriteGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus)
        {
            return writeGainSensor(ptr, gain, out outStatus);
        }
        public byte ReadExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus)
        {
            return readExternalSwitchSensor(ptr, out extSwInputOut, out outStatus);
        }
        public byte WriteExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus)
        {
            return writeExternalSwitchSensor(ptr, extSwInput, out outStatus);
        }
        public byte ReadColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus)
        {
            return readColorCallibri(ptr, out callibriColorOut, out outStatus);
        }
        public byte GetSupportedFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = getSupportedFiltersCountSensor(ptr);
            SensorFilter[] filtersArr = new SensorFilter[sz];
            var res = getSupportedFiltersSensor(ptr, filtersArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[outStatus.Success ? sz : 0];
            if (outStatus.Success)
            {
                Array.Copy(filtersArr, 0, filtersOut, 0, sz);
            }
            return res;
        }
        public byte IsSupportedFilterSensor(IntPtr ptr, SensorFilter filter)
        {
            return isSupportedFilterSensor(ptr, filter);
        }
        public byte ReadElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus)
        {
            return readElectrodeStateCallibri(ptr, out electrodeStateOut, out outStatus);
        }
        public byte ReadSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyEnvelopeSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public void ReadColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut)
        {
            readColorInfo(sensorInfo, out callibriColorOut);
        }
        public byte WriteFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus)
        {
            return writeFirmwareModeSensor(ptr, mode, out outStatus);
        }
        public byte WriteDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus)
        {
            return writeDataOffsetSensor(ptr, dataOffset, out outStatus);
        }
        public byte ReadADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus)
        {
            return readADCInputSensor(ptr, out adcInputOut, out outStatus);
        }
        public byte WriteADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus)
        {
            return writeADCInputSensor(ptr, adcInput, out outStatus);
        }

        public byte ReadStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus)
        {
            return readStimulatorAndMAStateCallibri(ptr, out stimulatorMAStateOut, out outStatus);
        }
        public byte ReadStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus)
        {
            return readStimulatorParamCallibri(ptr, out stimulationParamsOut, out outStatus);
        }
        public byte WriteStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus)
        {
            return writeStimulatorParamCallibri(ptr, stimulationParams, out outStatus);
        }
        public byte ReadMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus)
        {
            return readMotionAssistantParamCallibri(ptr, out motionAssistantParamsOut, out outStatus);
        }
        public byte WriteMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus)
        {
            return writeMotionAssistantParamCallibri(ptr, motionAssistantParams, out outStatus);
        }
        public byte ReadMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus)
        {
            return readMotionCounterParamCallibri(ptr, out motionCounterParamOut, out outStatus);
        }
        public byte WriteMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus)
        {
            return writeMotionCounterParamCallibri(ptr, motionCounterParam, out outStatus);
        }
        public byte ReadMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus)
        {
            return readMotionCounterCallibri(ptr, out motionCounterOut, out outStatus);
        }

        public byte GetSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus)
        {
            return getSignalSettingsCallibri(ptr, out callibriSignalTypeOut, out outStatus);
        }
        public byte SetSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus)
        {
            return setSignalSettingsCallibri(ptr, callibriSignalType, out outStatus);
        }
        public byte ReadMEMSCalibrateStateCallibri(IntPtr ptr, out bool state, out OpStatus outStatus)
        {
            byte bState;
            var res = readMEMSCalibrateStateCallibri(ptr, out bState, out outStatus);
            state = bState != 0;
            return res;
        }
        public byte ReadSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyRespSensor(ptr, out samplingFrequencyOut, out outStatus);
        }

        public byte AddSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackCallibri(IntPtr handle)
        {
            removeSignalCallbackCallibri(handle);
        }
        public byte AddRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addRespirationCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveRespirationCallbackCallibri(IntPtr handle)
        {
            removeRespirationCallbackCallibri(handle);
        }
        public byte AddElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addElectrodeStateCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveElectrodeStateCallbackCallibri(IntPtr handle)
        {
            removeElectrodeStateCallbackCallibri(handle);
        }
        public byte AddEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addEnvelopeDataCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveEnvelopeDataCallbackCallibri(IntPtr handle)
        {
            removeEnvelopeDataCallbackCallibri(handle);
        }

        public byte AddQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addQuaternionDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveQuaternionDataCallback(IntPtr handle)
        {
            removeQuaternionDataCallback(handle);
        }
        public byte AddResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit(IntPtr handle)
        {
            removeResistCallbackBrainBit(handle);
        }
        public byte AddSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalDataCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalDataCallbackBrainBit(IntPtr handle)
        {
            removeSignalDataCallbackBrainBit(handle);
        }
        public byte ReadSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyFPGSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readIrAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus)
        {
            return writeIrAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readRedAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus)
        {
            return writeRedAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus)
        {
            return readAmpMode(ptr, out modeOut, out outStatus);
        }

        public byte AddAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addAmpModeCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveAmpModeCallback(IntPtr handle)
        {
            removeAmpModeCallback(handle);
        }
	    public byte PingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus)
        {
            return pingNeuroSmart(ptr, marker, out outStatus);
        }
        public byte AddFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addFPGDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveFPGDataCallback(IntPtr handle)
        {
            removeFPGDataCallback(handle);
        }
	    public byte WriteSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus)
        {
            return writeSamplingFrequencySensor(ptr, samplingFrequency, out outStatus);
        }


        public byte AddBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addBatteryCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveBatteryCallback(IntPtr handle)
        {
            removeBatteryCallback(handle);
        }
        
        public byte AddConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addConnectionStateCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveConnectionStateCallback(IntPtr handle)
        {
            removeConnectionStateCallback(handle);
        }
                public byte AddSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackBrainBit2(IntPtr handle)
        {
            removeSignalCallbackBrainBit2(handle);
        }
        public byte AddResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit2(IntPtr handle)
        {
            removeResistCallbackBrainBit2(handle);
        }

        public byte ReadSupportedChannelsBrainBit2(IntPtr ptr, out EEGChannelInfo[] channelsOut, out OpStatus outStatus)
        {
            var cnt = getChannelsCountSensor(ptr);
            channelsOut = new EEGChannelInfo[cnt];
            var res = readSupportedChannelsBrainBit2(ptr, channelsOut, ref cnt, out outStatus);
            if (cnt != channelsOut.Length)
            {
                var chs = new EEGChannelInfo[cnt];
                Array.Copy(channelsOut, chs, cnt);
                channelsOut = chs;
            }
            return res;
        }
        public byte ReadAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus)
        {
            var res = readAmplifierParamBrainBit2(ptr, out ampParamOut, out outStatus);
            if (outStatus.Success)
            {
                var cnt = getChannelsCountSensor(ptr);
                if (ampParamOut.ChSignalMode.Length != cnt)
                {
                    var chSignalMode = new BrainBit2ChannelMode[cnt];
                    Array.Copy(ampParamOut.ChSignalMode, chSignalMode, Math.Min(cnt, ampParamOut.ChSignalMode.Length));
                    ampParamOut.ChSignalMode = chSignalMode;
                }
                if (ampParamOut.ChResistUse.Length != cnt)
                {
                    var chResistUse = new bool[cnt];
                    Array.Copy(ampParamOut.ChResistUse, chResistUse, Math.Min(cnt, ampParamOut.ChResistUse.Length));
                    ampParamOut.ChResistUse = chResistUse;
                }
                if (ampParamOut.ChGain.Length != cnt)
                {
                    var chGain = new SensorGain[cnt];
                    Array.Copy(ampParamOut.ChGain, chGain, Math.Min(cnt, ampParamOut.ChGain.Length));
                    ampParamOut.ChGain = chGain;
                }
            }
            return res;
        }
        public byte WriteAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus)
        {
            var cnt = SdkLibConst.BrainBit2MaxChCount;
            if (ampParam.ChSignalMode.Length != cnt)
            {
                var chSignalMode = new BrainBit2ChannelMode[cnt];
                Array.Copy(ampParam.ChSignalMode, chSignalMode, Math.Min(cnt, ampParam.ChSignalMode.Length));
                ampParam.ChSignalMode = chSignalMode;
            }
            if (ampParam.ChResistUse.Length != cnt)
            {
                var chResistUse = new bool[cnt];
                Array.Copy(ampParam.ChResistUse, chResistUse, Math.Min(cnt, ampParam.ChResistUse.Length));
                ampParam.ChResistUse = chResistUse;
            }
            if (ampParam.ChGain.Length != cnt)
            {
                var chGain = new SensorGain[cnt];
                Array.Copy(ampParam.ChGain, chGain, Math.Min(cnt, ampParam.ChGain.Length));
                ampParam.ChGain = chGain;
            }
            return writeAmplifierParamBrainBit2(ptr, ampParam, out outStatus);
        }

    }
    internal sealed class SDKApiArm64 : ISDKApi
    {
        public const string LibNameOS = SdkLib.LibName + "-arm64";
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createScanner(SensorFamily[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeScanner(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte startScanner(IntPtr ptr, out OpStatus outStatus, int numOfTrying);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte stopScanner(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte sensorsScanner(IntPtr ptr, [In, Out] SensorInfo[] sensors, ref int szSensorsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSensorsCallbackScanner(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte connectSensor(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte disconnectSensor(IntPtr ptr, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getFeaturesCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getCommandsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getParametersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getChannelsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte execCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern SensorFamily getFamilySensor(IntPtr ptr);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readNameSensor(IntPtr ptr, StringBuilder nameOut, int szNameIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeNameSensor(IntPtr ptr, StringBuilder name, int szName, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAddressSensor(IntPtr ptr, StringBuilder addressOut, int szAddressIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSerialNumberSensor(IntPtr ptr, StringBuilder serialNumberOut, int szSerialNumberIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSerialNumberSensor(IntPtr ptr, StringBuilder serialNumber, int szSerialNumber, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus);
        
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus);
                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeMEMSDataCallback(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus);
        
        		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  readAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus);

    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  addAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern void removeAmpModeCallback(IntPtr handle);
	[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  pingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  addFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeFPGDataCallback(IntPtr handle);
		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus);


        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeBatteryCallback(IntPtr handle);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeConnectionStateCallback(IntPtr handle);

                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readHardwareFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getSupportedFiltersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSupportedFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte isSupportedFilterSensor(IntPtr ptr, SensorFilter filter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void readColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte setSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMEMSCalibrateStateCallibri(IntPtr ptr, out byte state, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeRespirationCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeElectrodeStateCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeEnvelopeDataCallbackCallibri(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeQuaternionDataCallback(IntPtr handle);
	    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalDataCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSupportedChannelsBrainBit2(IntPtr ptr, [In, Out] EEGChannelInfo[] channelsOut, ref int szchannelsInOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus);


        public IntPtr CreateScanner(SensorFamily[] filters, out OpStatus outStatus)
        {
            return createScanner(filters, filters.Length, out outStatus);
        }
        public void FreeScanner(IntPtr ptr)
        {
            freeScanner(ptr);
        }
        public byte StartScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return startScanner(ptr, out outStatus, 1);
        }
        public byte StopScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return stopScanner(ptr, out outStatus);
        }
        public byte SensorsScanner(IntPtr ptr, out SensorInfo[] sensors, out OpStatus outStatus)
        {
            int sz = 64;
            SensorInfo[] sensorsArr = new SensorInfo[sz];
            var res = sensorsScanner(ptr, sensorsArr, ref sz, out outStatus);
            sensors = new SensorInfo[sz];
            Array.Copy(sensorsArr, 0, sensors, 0, sz);
            return res;
        }
        public byte AddSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSensorsCallbackScanner(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSensorsCallbackScanner(IntPtr handle)
        {
            removeSensorsCallbackScanner(handle);
        }
        public IntPtr CreateSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus)
        {
            return createSensor(ptr, sensor, out outStatus);
        }
        public void FreeSensor(IntPtr ptr)
        {
            freeSensor(ptr);
        }
        public byte ConnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return connectSensor(ptr, out outStatus);
        }
        public byte DisconnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return disconnectSensor(ptr, out outStatus);
        }

        public int GetFeaturesCountSensor(IntPtr ptr)
        {
            return getFeaturesCountSensor(ptr);
        }
        public byte GetFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus)
        {
            return getFeaturesSensor(ptr, sensorFeatures, ref szSensorFeaturesInOut, out outStatus);
        }
        public sbyte IsSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature)
        {
            return isSupportedFeatureSensor(ptr, sensorFeature);
        }

        public int GetCommandsCountSensor(IntPtr ptr)
        {
            return getCommandsCountSensor(ptr);
        }
        public byte GetCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus)
        {
            return getCommandsSensor(ptr, sensorCommands, ref szSensorCommandsInOut, out outStatus);
        }
        public sbyte IsSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand)
        {
            return isSupportedCommandSensor(ptr, sensorCommand);
        }

        public int GetParametersCountSensor(IntPtr ptr)
        {
            return getParametersCountSensor(ptr);
        }
        public byte GetParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus)
        {
            return getParametersSensor(ptr, sensorParameters, ref szSensorParametersInOut, out outStatus);
        }
        public sbyte IsSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter)
        {
            return isSupportedParameterSensor(ptr, sensorParameter);
        }

        public byte ExecCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus)
        {
            return execCommandSensor(ptr, sensorCommand, out outStatus);
        }
        public SensorFamily GetFamilySensor(IntPtr ptr)
        {
            return getFamilySensor(ptr);
        }

        public byte ReadNameSensor(IntPtr ptr, out string nameOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorNameLen);
            var res = readNameSensor(ptr, sb, sb.Capacity, out outStatus);
            nameOut = sb.ToString();
            return res;
        }
        public byte WriteNameSensor(IntPtr ptr, string name, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(name);
            return writeNameSensor(ptr, sb, sb.Capacity, out outStatus);
        }

        public byte ReadStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus)
        {
            return readStateSensor(ptr, out stateOut, out outStatus);
        }
        public byte ReadAddressSensor(IntPtr ptr, out string addressOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorAdrLen);
            var res = readAddressSensor(ptr, sb, sb.Capacity, out outStatus);
            addressOut = sb.ToString();
            return res;
        }
        public byte ReadSerialNumberSensor(IntPtr ptr, out string serialNumberOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorSNLen);
            var res = readSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
            serialNumberOut = sb.ToString();
            return res;
        }
        public byte WriteSerialNumberSensor(IntPtr ptr, string serialNumber, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(serialNumber);
            return writeSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
        }
        public byte ReadBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus)
        {
            return readBattPowerSensor(ptr, out battPowerOut, out outStatus);
        }
        
        public byte ReadSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencySensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        
        public byte ReadGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus)
        {
            return readGainSensor(ptr, out gainOut, out outStatus);
        }
        
        public byte ReadDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus)
        {
            return readDataOffsetSensor(ptr, out dataOffsetOut, out outStatus);
        }
                public byte ReadSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyMEMSSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus)
        {
            return readAccelerometerSensSensor(ptr, out accSensOut, out outStatus);
        }
        public byte WriteAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus)
        {
            return writeAccelerometerSensSensor(ptr, accSens, out outStatus);
        }
        public byte ReadGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus)
        {
            return readGyroscopeSensSensor(ptr, out gyroSensOut, out outStatus);
        }
        public byte WriteGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus)
        {
            return writeGyroscopeSensSensor(ptr, gyroSens, out outStatus);
        }
        public byte AddMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addMEMSDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveMEMSDataCallback(IntPtr handle)
        {
            removeMEMSDataCallback(handle);
        }

        public byte ReadFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus)
        {
            return readFirmwareModeSensor(ptr, out modeOut, out outStatus);
        }
        
        public byte ReadVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus)
        {
            return readVersionSensor(ptr, out versionOut, out outStatus);
        }
        	    public byte ReadSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyResistSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadHardwareFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = 64;
            SensorFilter[] sensorsArr = new SensorFilter[sz];
            var res = readHardwareFiltersSensor(ptr, sensorsArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[sz];
            Array.Copy(sensorsArr, 0, filtersOut, 0, sz);
            return res;
        }
        public byte WriteHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, out OpStatus outStatus)
        {
            return writeHardwareFiltersSensor(ptr, filters, filters.Length, out outStatus);
        }
        public byte WriteGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus)
        {
            return writeGainSensor(ptr, gain, out outStatus);
        }
        public byte ReadExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus)
        {
            return readExternalSwitchSensor(ptr, out extSwInputOut, out outStatus);
        }
        public byte WriteExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus)
        {
            return writeExternalSwitchSensor(ptr, extSwInput, out outStatus);
        }
        public byte ReadColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus)
        {
            return readColorCallibri(ptr, out callibriColorOut, out outStatus);
        }
        public byte GetSupportedFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = getSupportedFiltersCountSensor(ptr);
            SensorFilter[] filtersArr = new SensorFilter[sz];
            var res = getSupportedFiltersSensor(ptr, filtersArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[outStatus.Success ? sz : 0];
            if (outStatus.Success)
            {
                Array.Copy(filtersArr, 0, filtersOut, 0, sz);
            }
            return res;
        }
        public byte IsSupportedFilterSensor(IntPtr ptr, SensorFilter filter)
        {
            return isSupportedFilterSensor(ptr, filter);
        }
        public byte ReadElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus)
        {
            return readElectrodeStateCallibri(ptr, out electrodeStateOut, out outStatus);
        }
        public byte ReadSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyEnvelopeSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public void ReadColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut)
        {
            readColorInfo(sensorInfo, out callibriColorOut);
        }
        public byte WriteFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus)
        {
            return writeFirmwareModeSensor(ptr, mode, out outStatus);
        }
        public byte WriteDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus)
        {
            return writeDataOffsetSensor(ptr, dataOffset, out outStatus);
        }
        public byte ReadADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus)
        {
            return readADCInputSensor(ptr, out adcInputOut, out outStatus);
        }
        public byte WriteADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus)
        {
            return writeADCInputSensor(ptr, adcInput, out outStatus);
        }

        public byte ReadStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus)
        {
            return readStimulatorAndMAStateCallibri(ptr, out stimulatorMAStateOut, out outStatus);
        }
        public byte ReadStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus)
        {
            return readStimulatorParamCallibri(ptr, out stimulationParamsOut, out outStatus);
        }
        public byte WriteStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus)
        {
            return writeStimulatorParamCallibri(ptr, stimulationParams, out outStatus);
        }
        public byte ReadMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus)
        {
            return readMotionAssistantParamCallibri(ptr, out motionAssistantParamsOut, out outStatus);
        }
        public byte WriteMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus)
        {
            return writeMotionAssistantParamCallibri(ptr, motionAssistantParams, out outStatus);
        }
        public byte ReadMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus)
        {
            return readMotionCounterParamCallibri(ptr, out motionCounterParamOut, out outStatus);
        }
        public byte WriteMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus)
        {
            return writeMotionCounterParamCallibri(ptr, motionCounterParam, out outStatus);
        }
        public byte ReadMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus)
        {
            return readMotionCounterCallibri(ptr, out motionCounterOut, out outStatus);
        }

        public byte GetSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus)
        {
            return getSignalSettingsCallibri(ptr, out callibriSignalTypeOut, out outStatus);
        }
        public byte SetSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus)
        {
            return setSignalSettingsCallibri(ptr, callibriSignalType, out outStatus);
        }
        public byte ReadMEMSCalibrateStateCallibri(IntPtr ptr, out bool state, out OpStatus outStatus)
        {
            byte bState;
            var res = readMEMSCalibrateStateCallibri(ptr, out bState, out outStatus);
            state = bState != 0;
            return res;
        }
        public byte ReadSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyRespSensor(ptr, out samplingFrequencyOut, out outStatus);
        }

        public byte AddSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackCallibri(IntPtr handle)
        {
            removeSignalCallbackCallibri(handle);
        }
        public byte AddRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addRespirationCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveRespirationCallbackCallibri(IntPtr handle)
        {
            removeRespirationCallbackCallibri(handle);
        }
        public byte AddElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addElectrodeStateCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveElectrodeStateCallbackCallibri(IntPtr handle)
        {
            removeElectrodeStateCallbackCallibri(handle);
        }
        public byte AddEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addEnvelopeDataCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveEnvelopeDataCallbackCallibri(IntPtr handle)
        {
            removeEnvelopeDataCallbackCallibri(handle);
        }

        public byte AddQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addQuaternionDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveQuaternionDataCallback(IntPtr handle)
        {
            removeQuaternionDataCallback(handle);
        }
        public byte AddResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit(IntPtr handle)
        {
            removeResistCallbackBrainBit(handle);
        }
        public byte AddSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalDataCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalDataCallbackBrainBit(IntPtr handle)
        {
            removeSignalDataCallbackBrainBit(handle);
        }
        public byte ReadSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyFPGSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readIrAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus)
        {
            return writeIrAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readRedAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus)
        {
            return writeRedAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus)
        {
            return readAmpMode(ptr, out modeOut, out outStatus);
        }

        public byte AddAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addAmpModeCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveAmpModeCallback(IntPtr handle)
        {
            removeAmpModeCallback(handle);
        }
	    public byte PingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus)
        {
            return pingNeuroSmart(ptr, marker, out outStatus);
        }
        public byte AddFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addFPGDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveFPGDataCallback(IntPtr handle)
        {
            removeFPGDataCallback(handle);
        }
	    public byte WriteSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus)
        {
            return writeSamplingFrequencySensor(ptr, samplingFrequency, out outStatus);
        }


        public byte AddBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addBatteryCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveBatteryCallback(IntPtr handle)
        {
            removeBatteryCallback(handle);
        }
        
        public byte AddConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addConnectionStateCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveConnectionStateCallback(IntPtr handle)
        {
            removeConnectionStateCallback(handle);
        }
                public byte AddSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackBrainBit2(IntPtr handle)
        {
            removeSignalCallbackBrainBit2(handle);
        }
        public byte AddResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit2(IntPtr handle)
        {
            removeResistCallbackBrainBit2(handle);
        }

        public byte ReadSupportedChannelsBrainBit2(IntPtr ptr, out EEGChannelInfo[] channelsOut, out OpStatus outStatus)
        {
            var cnt = getChannelsCountSensor(ptr);
            channelsOut = new EEGChannelInfo[cnt];
            var res = readSupportedChannelsBrainBit2(ptr, channelsOut, ref cnt, out outStatus);
            if (cnt != channelsOut.Length)
            {
                var chs = new EEGChannelInfo[cnt];
                Array.Copy(channelsOut, chs, cnt);
                channelsOut = chs;
            }
            return res;
        }
        public byte ReadAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus)
        {
            var res = readAmplifierParamBrainBit2(ptr, out ampParamOut, out outStatus);
            if (outStatus.Success)
            {
                var cnt = getChannelsCountSensor(ptr);
                if (ampParamOut.ChSignalMode.Length != cnt)
                {
                    var chSignalMode = new BrainBit2ChannelMode[cnt];
                    Array.Copy(ampParamOut.ChSignalMode, chSignalMode, Math.Min(cnt, ampParamOut.ChSignalMode.Length));
                    ampParamOut.ChSignalMode = chSignalMode;
                }
                if (ampParamOut.ChResistUse.Length != cnt)
                {
                    var chResistUse = new bool[cnt];
                    Array.Copy(ampParamOut.ChResistUse, chResistUse, Math.Min(cnt, ampParamOut.ChResistUse.Length));
                    ampParamOut.ChResistUse = chResistUse;
                }
                if (ampParamOut.ChGain.Length != cnt)
                {
                    var chGain = new SensorGain[cnt];
                    Array.Copy(ampParamOut.ChGain, chGain, Math.Min(cnt, ampParamOut.ChGain.Length));
                    ampParamOut.ChGain = chGain;
                }
            }
            return res;
        }
        public byte WriteAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus)
        {
            var cnt = SdkLibConst.BrainBit2MaxChCount;
            if (ampParam.ChSignalMode.Length != cnt)
            {
                var chSignalMode = new BrainBit2ChannelMode[cnt];
                Array.Copy(ampParam.ChSignalMode, chSignalMode, Math.Min(cnt, ampParam.ChSignalMode.Length));
                ampParam.ChSignalMode = chSignalMode;
            }
            if (ampParam.ChResistUse.Length != cnt)
            {
                var chResistUse = new bool[cnt];
                Array.Copy(ampParam.ChResistUse, chResistUse, Math.Min(cnt, ampParam.ChResistUse.Length));
                ampParam.ChResistUse = chResistUse;
            }
            if (ampParam.ChGain.Length != cnt)
            {
                var chGain = new SensorGain[cnt];
                Array.Copy(ampParam.ChGain, chGain, Math.Min(cnt, ampParam.ChGain.Length));
                ampParam.ChGain = chGain;
            }
            return writeAmplifierParamBrainBit2(ptr, ampParam, out outStatus);
        }

    }
    internal sealed class SDKApiAllArch : ISDKApi
    {
        public const string LibNameOS = SdkLib.LibName;
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createScanner(SensorFamily[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeScanner(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte startScanner(IntPtr ptr, out OpStatus outStatus, int numOfTrying);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte stopScanner(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte sensorsScanner(IntPtr ptr, [In, Out] SensorInfo[] sensors, ref int szSensorsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSensorsCallbackScanner(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte connectSensor(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte disconnectSensor(IntPtr ptr, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getFeaturesCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getCommandsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getParametersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getChannelsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte execCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern SensorFamily getFamilySensor(IntPtr ptr);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readNameSensor(IntPtr ptr, StringBuilder nameOut, int szNameIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeNameSensor(IntPtr ptr, StringBuilder name, int szName, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAddressSensor(IntPtr ptr, StringBuilder addressOut, int szAddressIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSerialNumberSensor(IntPtr ptr, StringBuilder serialNumberOut, int szSerialNumberIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSerialNumberSensor(IntPtr ptr, StringBuilder serialNumber, int szSerialNumber, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus);
        
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus);
                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeMEMSDataCallback(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus);
        
        		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  readAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus);

    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  addAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern void removeAmpModeCallback(IntPtr handle);
	[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  pingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  addFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeFPGDataCallback(IntPtr handle);
		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus);


        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeBatteryCallback(IntPtr handle);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeConnectionStateCallback(IntPtr handle);

                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readHardwareFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getSupportedFiltersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSupportedFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte isSupportedFilterSensor(IntPtr ptr, SensorFilter filter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void readColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte setSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMEMSCalibrateStateCallibri(IntPtr ptr, out byte state, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeRespirationCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeElectrodeStateCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeEnvelopeDataCallbackCallibri(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeQuaternionDataCallback(IntPtr handle);
	    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalDataCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSupportedChannelsBrainBit2(IntPtr ptr, [In, Out] EEGChannelInfo[] channelsOut, ref int szchannelsInOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus);


        public IntPtr CreateScanner(SensorFamily[] filters, out OpStatus outStatus)
        {
            return createScanner(filters, filters.Length, out outStatus);
        }
        public void FreeScanner(IntPtr ptr)
        {
            freeScanner(ptr);
        }
        public byte StartScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return startScanner(ptr, out outStatus, 1);
        }
        public byte StopScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return stopScanner(ptr, out outStatus);
        }
        public byte SensorsScanner(IntPtr ptr, out SensorInfo[] sensors, out OpStatus outStatus)
        {
            int sz = 64;
            SensorInfo[] sensorsArr = new SensorInfo[sz];
            var res = sensorsScanner(ptr, sensorsArr, ref sz, out outStatus);
            sensors = new SensorInfo[sz];
            Array.Copy(sensorsArr, 0, sensors, 0, sz);
            return res;
        }
        public byte AddSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSensorsCallbackScanner(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSensorsCallbackScanner(IntPtr handle)
        {
            removeSensorsCallbackScanner(handle);
        }
        public IntPtr CreateSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus)
        {
            return createSensor(ptr, sensor, out outStatus);
        }
        public void FreeSensor(IntPtr ptr)
        {
            freeSensor(ptr);
        }
        public byte ConnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return connectSensor(ptr, out outStatus);
        }
        public byte DisconnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return disconnectSensor(ptr, out outStatus);
        }

        public int GetFeaturesCountSensor(IntPtr ptr)
        {
            return getFeaturesCountSensor(ptr);
        }
        public byte GetFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus)
        {
            return getFeaturesSensor(ptr, sensorFeatures, ref szSensorFeaturesInOut, out outStatus);
        }
        public sbyte IsSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature)
        {
            return isSupportedFeatureSensor(ptr, sensorFeature);
        }

        public int GetCommandsCountSensor(IntPtr ptr)
        {
            return getCommandsCountSensor(ptr);
        }
        public byte GetCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus)
        {
            return getCommandsSensor(ptr, sensorCommands, ref szSensorCommandsInOut, out outStatus);
        }
        public sbyte IsSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand)
        {
            return isSupportedCommandSensor(ptr, sensorCommand);
        }

        public int GetParametersCountSensor(IntPtr ptr)
        {
            return getParametersCountSensor(ptr);
        }
        public byte GetParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus)
        {
            return getParametersSensor(ptr, sensorParameters, ref szSensorParametersInOut, out outStatus);
        }
        public sbyte IsSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter)
        {
            return isSupportedParameterSensor(ptr, sensorParameter);
        }

        public byte ExecCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus)
        {
            return execCommandSensor(ptr, sensorCommand, out outStatus);
        }
        public SensorFamily GetFamilySensor(IntPtr ptr)
        {
            return getFamilySensor(ptr);
        }

        public byte ReadNameSensor(IntPtr ptr, out string nameOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorNameLen);
            var res = readNameSensor(ptr, sb, sb.Capacity, out outStatus);
            nameOut = sb.ToString();
            return res;
        }
        public byte WriteNameSensor(IntPtr ptr, string name, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(name);
            return writeNameSensor(ptr, sb, sb.Capacity, out outStatus);
        }

        public byte ReadStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus)
        {
            return readStateSensor(ptr, out stateOut, out outStatus);
        }
        public byte ReadAddressSensor(IntPtr ptr, out string addressOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorAdrLen);
            var res = readAddressSensor(ptr, sb, sb.Capacity, out outStatus);
            addressOut = sb.ToString();
            return res;
        }
        public byte ReadSerialNumberSensor(IntPtr ptr, out string serialNumberOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorSNLen);
            var res = readSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
            serialNumberOut = sb.ToString();
            return res;
        }
        public byte WriteSerialNumberSensor(IntPtr ptr, string serialNumber, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(serialNumber);
            return writeSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
        }
        public byte ReadBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus)
        {
            return readBattPowerSensor(ptr, out battPowerOut, out outStatus);
        }
        
        public byte ReadSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencySensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        
        public byte ReadGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus)
        {
            return readGainSensor(ptr, out gainOut, out outStatus);
        }
        
        public byte ReadDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus)
        {
            return readDataOffsetSensor(ptr, out dataOffsetOut, out outStatus);
        }
                public byte ReadSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyMEMSSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus)
        {
            return readAccelerometerSensSensor(ptr, out accSensOut, out outStatus);
        }
        public byte WriteAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus)
        {
            return writeAccelerometerSensSensor(ptr, accSens, out outStatus);
        }
        public byte ReadGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus)
        {
            return readGyroscopeSensSensor(ptr, out gyroSensOut, out outStatus);
        }
        public byte WriteGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus)
        {
            return writeGyroscopeSensSensor(ptr, gyroSens, out outStatus);
        }
        public byte AddMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addMEMSDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveMEMSDataCallback(IntPtr handle)
        {
            removeMEMSDataCallback(handle);
        }

        public byte ReadFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus)
        {
            return readFirmwareModeSensor(ptr, out modeOut, out outStatus);
        }
        
        public byte ReadVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus)
        {
            return readVersionSensor(ptr, out versionOut, out outStatus);
        }
        	    public byte ReadSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyResistSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadHardwareFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = 64;
            SensorFilter[] sensorsArr = new SensorFilter[sz];
            var res = readHardwareFiltersSensor(ptr, sensorsArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[sz];
            Array.Copy(sensorsArr, 0, filtersOut, 0, sz);
            return res;
        }
        public byte WriteHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, out OpStatus outStatus)
        {
            return writeHardwareFiltersSensor(ptr, filters, filters.Length, out outStatus);
        }
        public byte WriteGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus)
        {
            return writeGainSensor(ptr, gain, out outStatus);
        }
        public byte ReadExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus)
        {
            return readExternalSwitchSensor(ptr, out extSwInputOut, out outStatus);
        }
        public byte WriteExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus)
        {
            return writeExternalSwitchSensor(ptr, extSwInput, out outStatus);
        }
        public byte ReadColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus)
        {
            return readColorCallibri(ptr, out callibriColorOut, out outStatus);
        }
        public byte GetSupportedFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = getSupportedFiltersCountSensor(ptr);
            SensorFilter[] filtersArr = new SensorFilter[sz];
            var res = getSupportedFiltersSensor(ptr, filtersArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[outStatus.Success ? sz : 0];
            if (outStatus.Success)
            {
                Array.Copy(filtersArr, 0, filtersOut, 0, sz);
            }
            return res;
        }
        public byte IsSupportedFilterSensor(IntPtr ptr, SensorFilter filter)
        {
            return isSupportedFilterSensor(ptr, filter);
        }
        public byte ReadElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus)
        {
            return readElectrodeStateCallibri(ptr, out electrodeStateOut, out outStatus);
        }
        public byte ReadSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyEnvelopeSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public void ReadColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut)
        {
            readColorInfo(sensorInfo, out callibriColorOut);
        }
        public byte WriteFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus)
        {
            return writeFirmwareModeSensor(ptr, mode, out outStatus);
        }
        public byte WriteDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus)
        {
            return writeDataOffsetSensor(ptr, dataOffset, out outStatus);
        }
        public byte ReadADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus)
        {
            return readADCInputSensor(ptr, out adcInputOut, out outStatus);
        }
        public byte WriteADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus)
        {
            return writeADCInputSensor(ptr, adcInput, out outStatus);
        }

        public byte ReadStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus)
        {
            return readStimulatorAndMAStateCallibri(ptr, out stimulatorMAStateOut, out outStatus);
        }
        public byte ReadStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus)
        {
            return readStimulatorParamCallibri(ptr, out stimulationParamsOut, out outStatus);
        }
        public byte WriteStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus)
        {
            return writeStimulatorParamCallibri(ptr, stimulationParams, out outStatus);
        }
        public byte ReadMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus)
        {
            return readMotionAssistantParamCallibri(ptr, out motionAssistantParamsOut, out outStatus);
        }
        public byte WriteMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus)
        {
            return writeMotionAssistantParamCallibri(ptr, motionAssistantParams, out outStatus);
        }
        public byte ReadMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus)
        {
            return readMotionCounterParamCallibri(ptr, out motionCounterParamOut, out outStatus);
        }
        public byte WriteMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus)
        {
            return writeMotionCounterParamCallibri(ptr, motionCounterParam, out outStatus);
        }
        public byte ReadMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus)
        {
            return readMotionCounterCallibri(ptr, out motionCounterOut, out outStatus);
        }

        public byte GetSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus)
        {
            return getSignalSettingsCallibri(ptr, out callibriSignalTypeOut, out outStatus);
        }
        public byte SetSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus)
        {
            return setSignalSettingsCallibri(ptr, callibriSignalType, out outStatus);
        }
        public byte ReadMEMSCalibrateStateCallibri(IntPtr ptr, out bool state, out OpStatus outStatus)
        {
            byte bState;
            var res = readMEMSCalibrateStateCallibri(ptr, out bState, out outStatus);
            state = bState != 0;
            return res;
        }
        public byte ReadSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyRespSensor(ptr, out samplingFrequencyOut, out outStatus);
        }

        public byte AddSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackCallibri(IntPtr handle)
        {
            removeSignalCallbackCallibri(handle);
        }
        public byte AddRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addRespirationCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveRespirationCallbackCallibri(IntPtr handle)
        {
            removeRespirationCallbackCallibri(handle);
        }
        public byte AddElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addElectrodeStateCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveElectrodeStateCallbackCallibri(IntPtr handle)
        {
            removeElectrodeStateCallbackCallibri(handle);
        }
        public byte AddEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addEnvelopeDataCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveEnvelopeDataCallbackCallibri(IntPtr handle)
        {
            removeEnvelopeDataCallbackCallibri(handle);
        }

        public byte AddQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addQuaternionDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveQuaternionDataCallback(IntPtr handle)
        {
            removeQuaternionDataCallback(handle);
        }
        public byte AddResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit(IntPtr handle)
        {
            removeResistCallbackBrainBit(handle);
        }
        public byte AddSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalDataCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalDataCallbackBrainBit(IntPtr handle)
        {
            removeSignalDataCallbackBrainBit(handle);
        }
        public byte ReadSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyFPGSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readIrAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus)
        {
            return writeIrAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readRedAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus)
        {
            return writeRedAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus)
        {
            return readAmpMode(ptr, out modeOut, out outStatus);
        }

        public byte AddAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addAmpModeCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveAmpModeCallback(IntPtr handle)
        {
            removeAmpModeCallback(handle);
        }
	    public byte PingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus)
        {
            return pingNeuroSmart(ptr, marker, out outStatus);
        }
        public byte AddFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addFPGDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveFPGDataCallback(IntPtr handle)
        {
            removeFPGDataCallback(handle);
        }
	    public byte WriteSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus)
        {
            return writeSamplingFrequencySensor(ptr, samplingFrequency, out outStatus);
        }


        public byte AddBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addBatteryCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveBatteryCallback(IntPtr handle)
        {
            removeBatteryCallback(handle);
        }
        
        public byte AddConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addConnectionStateCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveConnectionStateCallback(IntPtr handle)
        {
            removeConnectionStateCallback(handle);
        }
                public byte AddSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackBrainBit2(IntPtr handle)
        {
            removeSignalCallbackBrainBit2(handle);
        }
        public byte AddResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit2(IntPtr handle)
        {
            removeResistCallbackBrainBit2(handle);
        }

        public byte ReadSupportedChannelsBrainBit2(IntPtr ptr, out EEGChannelInfo[] channelsOut, out OpStatus outStatus)
        {
            var cnt = getChannelsCountSensor(ptr);
            channelsOut = new EEGChannelInfo[cnt];
            var res = readSupportedChannelsBrainBit2(ptr, channelsOut, ref cnt, out outStatus);
            if (cnt != channelsOut.Length)
            {
                var chs = new EEGChannelInfo[cnt];
                Array.Copy(channelsOut, chs, cnt);
                channelsOut = chs;
            }
            return res;
        }
        public byte ReadAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus)
        {
            var res = readAmplifierParamBrainBit2(ptr, out ampParamOut, out outStatus);
            if (outStatus.Success)
            {
                var cnt = getChannelsCountSensor(ptr);
                if (ampParamOut.ChSignalMode.Length != cnt)
                {
                    var chSignalMode = new BrainBit2ChannelMode[cnt];
                    Array.Copy(ampParamOut.ChSignalMode, chSignalMode, Math.Min(cnt, ampParamOut.ChSignalMode.Length));
                    ampParamOut.ChSignalMode = chSignalMode;
                }
                if (ampParamOut.ChResistUse.Length != cnt)
                {
                    var chResistUse = new bool[cnt];
                    Array.Copy(ampParamOut.ChResistUse, chResistUse, Math.Min(cnt, ampParamOut.ChResistUse.Length));
                    ampParamOut.ChResistUse = chResistUse;
                }
                if (ampParamOut.ChGain.Length != cnt)
                {
                    var chGain = new SensorGain[cnt];
                    Array.Copy(ampParamOut.ChGain, chGain, Math.Min(cnt, ampParamOut.ChGain.Length));
                    ampParamOut.ChGain = chGain;
                }
            }
            return res;
        }
        public byte WriteAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus)
        {
            var cnt = SdkLibConst.BrainBit2MaxChCount;
            if (ampParam.ChSignalMode.Length != cnt)
            {
                var chSignalMode = new BrainBit2ChannelMode[cnt];
                Array.Copy(ampParam.ChSignalMode, chSignalMode, Math.Min(cnt, ampParam.ChSignalMode.Length));
                ampParam.ChSignalMode = chSignalMode;
            }
            if (ampParam.ChResistUse.Length != cnt)
            {
                var chResistUse = new bool[cnt];
                Array.Copy(ampParam.ChResistUse, chResistUse, Math.Min(cnt, ampParam.ChResistUse.Length));
                ampParam.ChResistUse = chResistUse;
            }
            if (ampParam.ChGain.Length != cnt)
            {
                var chGain = new SensorGain[cnt];
                Array.Copy(ampParam.ChGain, chGain, Math.Min(cnt, ampParam.ChGain.Length));
                ampParam.ChGain = chGain;
            }
            return writeAmplifierParamBrainBit2(ptr, ampParam, out outStatus);
        }

    }
#else
    internal sealed class SDKApiIOS : ISDKApi
    {
        public const string LibNameOS = SdkLib.LibNameIOS;
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createScanner(SensorFamily[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeScanner(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte startScanner(IntPtr ptr, out OpStatus outStatus, int numOfTrying);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte stopScanner(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte sensorsScanner(IntPtr ptr, [In, Out] SensorInfo[] sensors, ref int szSensorsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSensorsCallbackScanner(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void freeSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte connectSensor(IntPtr ptr, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte disconnectSensor(IntPtr ptr, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getFeaturesCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getCommandsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getParametersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern sbyte isSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getChannelsCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte execCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern SensorFamily getFamilySensor(IntPtr ptr);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readNameSensor(IntPtr ptr, StringBuilder nameOut, int szNameIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeNameSensor(IntPtr ptr, StringBuilder name, int szName, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAddressSensor(IntPtr ptr, StringBuilder addressOut, int szAddressIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSerialNumberSensor(IntPtr ptr, StringBuilder serialNumberOut, int szSerialNumberIn, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSerialNumberSensor(IntPtr ptr, StringBuilder serialNumber, int szSerialNumber, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus);
        
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus);
                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeMEMSDataCallback(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus);
        
        		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  readAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus);

    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  addAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern void removeAmpModeCallback(IntPtr handle);
	[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
    private static extern byte  pingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  addFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeFPGDataCallback(IntPtr handle);
		[DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus);


        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeBatteryCallback(IntPtr handle);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeConnectionStateCallback(IntPtr handle);

                [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readHardwareFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, int szFilters, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus);
        
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern int getSupportedFiltersCountSensor(IntPtr ptr);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSupportedFiltersSensor(IntPtr ptr, [In, Out] SensorFilter[] filtersOut, ref int szFiltersInOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte isSupportedFilterSensor(IntPtr ptr, SensorFilter filter);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void readColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte getSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte setSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readMEMSCalibrateStateCallibri(IntPtr ptr, out byte state, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeRespirationCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeElectrodeStateCallbackCallibri(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeEnvelopeDataCallbackCallibri(IntPtr handle);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeQuaternionDataCallback(IntPtr handle);
	    [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalDataCallbackBrainBit(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte readSupportedChannelsBrainBit2(IntPtr ptr, [In, Out] EEGChannelInfo[] channelsOut, ref int szchannelsInOut, out OpStatus outStatus);

        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeSignalCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte addResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeResistCallbackBrainBit2(IntPtr handle);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte  readAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus);
        [DllImport(LibNameOS, CallingConvention = CallingConvention.Cdecl)]
        private static extern byte writeAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus);


        public IntPtr CreateScanner(SensorFamily[] filters, out OpStatus outStatus)
        {
            return createScanner(filters, filters.Length, out outStatus);
        }
        public void FreeScanner(IntPtr ptr)
        {
            freeScanner(ptr);
        }
        public byte StartScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return startScanner(ptr, out outStatus, 1);
        }
        public byte StopScanner(IntPtr ptr, out OpStatus outStatus)
        {
            return stopScanner(ptr, out outStatus);
        }
        public byte SensorsScanner(IntPtr ptr, out SensorInfo[] sensors, out OpStatus outStatus)
        {
            int sz = 64;
            SensorInfo[] sensorsArr = new SensorInfo[sz];
            var res = sensorsScanner(ptr, sensorsArr, ref sz, out outStatus);
            sensors = new SensorInfo[sz];
            Array.Copy(sensorsArr, 0, sensors, 0, sz);
            return res;
        }
        public byte AddSensorsCallbackScanner(IntPtr ptr, SensorsCallbackScanner callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSensorsCallbackScanner(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSensorsCallbackScanner(IntPtr handle)
        {
            removeSensorsCallbackScanner(handle);
        }
        public IntPtr CreateSensor(IntPtr ptr, SensorInfo sensor, out OpStatus outStatus)
        {
            return createSensor(ptr, sensor, out outStatus);
        }
        public void FreeSensor(IntPtr ptr)
        {
            freeSensor(ptr);
        }
        public byte ConnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return connectSensor(ptr, out outStatus);
        }
        public byte DisconnectSensor(IntPtr ptr, out OpStatus outStatus)
        {
            return disconnectSensor(ptr, out outStatus);
        }

        public int GetFeaturesCountSensor(IntPtr ptr)
        {
            return getFeaturesCountSensor(ptr);
        }
        public byte GetFeaturesSensor(IntPtr ptr, [In, Out] SensorFeature[] sensorFeatures, ref int szSensorFeaturesInOut, out OpStatus outStatus)
        {
            return getFeaturesSensor(ptr, sensorFeatures, ref szSensorFeaturesInOut, out outStatus);
        }
        public sbyte IsSupportedFeatureSensor(IntPtr ptr, SensorFeature sensorFeature)
        {
            return isSupportedFeatureSensor(ptr, sensorFeature);
        }

        public int GetCommandsCountSensor(IntPtr ptr)
        {
            return getCommandsCountSensor(ptr);
        }
        public byte GetCommandsSensor(IntPtr ptr, [In, Out] SensorCommand[] sensorCommands, ref int szSensorCommandsInOut, out OpStatus outStatus)
        {
            return getCommandsSensor(ptr, sensorCommands, ref szSensorCommandsInOut, out outStatus);
        }
        public sbyte IsSupportedCommandSensor(IntPtr ptr, SensorCommand sensorCommand)
        {
            return isSupportedCommandSensor(ptr, sensorCommand);
        }

        public int GetParametersCountSensor(IntPtr ptr)
        {
            return getParametersCountSensor(ptr);
        }
        public byte GetParametersSensor(IntPtr ptr, [In, Out] ParameterInfo[] sensorParameters, ref int szSensorParametersInOut, out OpStatus outStatus)
        {
            return getParametersSensor(ptr, sensorParameters, ref szSensorParametersInOut, out outStatus);
        }
        public sbyte IsSupportedParameterSensor(IntPtr ptr, SensorParameter sensorParameter)
        {
            return isSupportedParameterSensor(ptr, sensorParameter);
        }

        public byte ExecCommandSensor(IntPtr ptr, SensorCommand sensorCommand, out OpStatus outStatus)
        {
            return execCommandSensor(ptr, sensorCommand, out outStatus);
        }
        public SensorFamily GetFamilySensor(IntPtr ptr)
        {
            return getFamilySensor(ptr);
        }

        public byte ReadNameSensor(IntPtr ptr, out string nameOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorNameLen);
            var res = readNameSensor(ptr, sb, sb.Capacity, out outStatus);
            nameOut = sb.ToString();
            return res;
        }
        public byte WriteNameSensor(IntPtr ptr, string name, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(name);
            return writeNameSensor(ptr, sb, sb.Capacity, out outStatus);
        }

        public byte ReadStateSensor(IntPtr ptr, out SensorState stateOut, out OpStatus outStatus)
        {
            return readStateSensor(ptr, out stateOut, out outStatus);
        }
        public byte ReadAddressSensor(IntPtr ptr, out string addressOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorAdrLen);
            var res = readAddressSensor(ptr, sb, sb.Capacity, out outStatus);
            addressOut = sb.ToString();
            return res;
        }
        public byte ReadSerialNumberSensor(IntPtr ptr, out string serialNumberOut, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(SdkLibConst.SensorSNLen);
            var res = readSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
            serialNumberOut = sb.ToString();
            return res;
        }
        public byte WriteSerialNumberSensor(IntPtr ptr, string serialNumber, out OpStatus outStatus)
        {
            StringBuilder sb = new StringBuilder(serialNumber);
            return writeSerialNumberSensor(ptr, sb, sb.Capacity, out outStatus);
        }
        public byte ReadBattPowerSensor(IntPtr ptr, out int battPowerOut, out OpStatus outStatus)
        {
            return readBattPowerSensor(ptr, out battPowerOut, out outStatus);
        }
        
        public byte ReadSamplingFrequencySensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencySensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        
        public byte ReadGainSensor(IntPtr ptr, out SensorGain gainOut, out OpStatus outStatus)
        {
            return readGainSensor(ptr, out gainOut, out outStatus);
        }
        
        public byte ReadDataOffsetSensor(IntPtr ptr, out SensorDataOffset dataOffsetOut, out OpStatus outStatus)
        {
            return readDataOffsetSensor(ptr, out dataOffsetOut, out outStatus);
        }
                public byte ReadSamplingFrequencyMEMSSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyMEMSSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadAccelerometerSensSensor(IntPtr ptr, out SensorAccelerometerSensitivity accSensOut, out OpStatus outStatus)
        {
            return readAccelerometerSensSensor(ptr, out accSensOut, out outStatus);
        }
        public byte WriteAccelerometerSensSensor(IntPtr ptr, SensorAccelerometerSensitivity accSens, out OpStatus outStatus)
        {
            return writeAccelerometerSensSensor(ptr, accSens, out outStatus);
        }
        public byte ReadGyroscopeSensSensor(IntPtr ptr, out SensorGyroscopeSensitivity gyroSensOut, out OpStatus outStatus)
        {
            return readGyroscopeSensSensor(ptr, out gyroSensOut, out outStatus);
        }
        public byte WriteGyroscopeSensSensor(IntPtr ptr, SensorGyroscopeSensitivity gyroSens, out OpStatus outStatus)
        {
            return writeGyroscopeSensSensor(ptr, gyroSens, out outStatus);
        }
        public byte AddMEMSDataCallback(IntPtr ptr, MEMSDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addMEMSDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveMEMSDataCallback(IntPtr handle)
        {
            removeMEMSDataCallback(handle);
        }

        public byte ReadFirmwareModeSensor(IntPtr ptr, out SensorFirmwareMode modeOut, out OpStatus outStatus)
        {
            return readFirmwareModeSensor(ptr, out modeOut, out outStatus);
        }
        
        public byte ReadVersionSensor(IntPtr ptr, out SensorVersion versionOut, out OpStatus outStatus)
        {
            return readVersionSensor(ptr, out versionOut, out outStatus);
        }
        	    public byte ReadSamplingFrequencyResistSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyResistSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadHardwareFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = 64;
            SensorFilter[] sensorsArr = new SensorFilter[sz];
            var res = readHardwareFiltersSensor(ptr, sensorsArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[sz];
            Array.Copy(sensorsArr, 0, filtersOut, 0, sz);
            return res;
        }
        public byte WriteHardwareFiltersSensor(IntPtr ptr, SensorFilter[] filters, out OpStatus outStatus)
        {
            return writeHardwareFiltersSensor(ptr, filters, filters.Length, out outStatus);
        }
        public byte WriteGainSensor(IntPtr ptr, SensorGain gain, out OpStatus outStatus)
        {
            return writeGainSensor(ptr, gain, out outStatus);
        }
        public byte ReadExternalSwitchSensor(IntPtr ptr, out SensorExternalSwitchInput extSwInputOut, out OpStatus outStatus)
        {
            return readExternalSwitchSensor(ptr, out extSwInputOut, out outStatus);
        }
        public byte WriteExternalSwitchSensor(IntPtr ptr, SensorExternalSwitchInput extSwInput, out OpStatus outStatus)
        {
            return writeExternalSwitchSensor(ptr, extSwInput, out outStatus);
        }
        public byte ReadColorCallibri(IntPtr ptr, out CallibriColorType callibriColorOut, out OpStatus outStatus)
        {
            return readColorCallibri(ptr, out callibriColorOut, out outStatus);
        }
        public byte GetSupportedFiltersSensor(IntPtr ptr, out SensorFilter[] filtersOut, out OpStatus outStatus)
        {
            int sz = getSupportedFiltersCountSensor(ptr);
            SensorFilter[] filtersArr = new SensorFilter[sz];
            var res = getSupportedFiltersSensor(ptr, filtersArr, ref sz, out outStatus);
            filtersOut = new SensorFilter[outStatus.Success ? sz : 0];
            if (outStatus.Success)
            {
                Array.Copy(filtersArr, 0, filtersOut, 0, sz);
            }
            return res;
        }
        public byte IsSupportedFilterSensor(IntPtr ptr, SensorFilter filter)
        {
            return isSupportedFilterSensor(ptr, filter);
        }
        public byte ReadElectrodeStateCallibri(IntPtr ptr, out CallibriElectrodeState electrodeStateOut, out OpStatus outStatus)
        {
            return readElectrodeStateCallibri(ptr, out electrodeStateOut, out outStatus);
        }
        public byte ReadSamplingFrequencyEnvelopeSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyEnvelopeSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public void ReadColorInfo(SensorInfo sensorInfo, out CallibriColorType callibriColorOut)
        {
            readColorInfo(sensorInfo, out callibriColorOut);
        }
        public byte WriteFirmwareModeSensor(IntPtr ptr, SensorFirmwareMode mode, out OpStatus outStatus)
        {
            return writeFirmwareModeSensor(ptr, mode, out outStatus);
        }
        public byte WriteDataOffsetSensor(IntPtr ptr, SensorDataOffset dataOffset, out OpStatus outStatus)
        {
            return writeDataOffsetSensor(ptr, dataOffset, out outStatus);
        }
        public byte ReadADCInputSensor(IntPtr ptr, out SensorADCInput adcInputOut, out OpStatus outStatus)
        {
            return readADCInputSensor(ptr, out adcInputOut, out outStatus);
        }
        public byte WriteADCInputSensor(IntPtr ptr, SensorADCInput adcInput, out OpStatus outStatus)
        {
            return writeADCInputSensor(ptr, adcInput, out outStatus);
        }

        public byte ReadStimulatorAndMAStateCallibri(IntPtr ptr, out CallibriStimulatorMAState stimulatorMAStateOut, out OpStatus outStatus)
        {
            return readStimulatorAndMAStateCallibri(ptr, out stimulatorMAStateOut, out outStatus);
        }
        public byte ReadStimulatorParamCallibri(IntPtr ptr, out CallibriStimulationParams stimulationParamsOut, out OpStatus outStatus)
        {
            return readStimulatorParamCallibri(ptr, out stimulationParamsOut, out outStatus);
        }
        public byte WriteStimulatorParamCallibri(IntPtr ptr, CallibriStimulationParams stimulationParams, out OpStatus outStatus)
        {
            return writeStimulatorParamCallibri(ptr, stimulationParams, out outStatus);
        }
        public byte ReadMotionAssistantParamCallibri(IntPtr ptr, out CallibriMotionAssistantParams motionAssistantParamsOut, out OpStatus outStatus)
        {
            return readMotionAssistantParamCallibri(ptr, out motionAssistantParamsOut, out outStatus);
        }
        public byte WriteMotionAssistantParamCallibri(IntPtr ptr, CallibriMotionAssistantParams motionAssistantParams, out OpStatus outStatus)
        {
            return writeMotionAssistantParamCallibri(ptr, motionAssistantParams, out outStatus);
        }
        public byte ReadMotionCounterParamCallibri(IntPtr ptr, out CallibriMotionCounterParam motionCounterParamOut, out OpStatus outStatus)
        {
            return readMotionCounterParamCallibri(ptr, out motionCounterParamOut, out outStatus);
        }
        public byte WriteMotionCounterParamCallibri(IntPtr ptr, CallibriMotionCounterParam motionCounterParam, out OpStatus outStatus)
        {
            return writeMotionCounterParamCallibri(ptr, motionCounterParam, out outStatus);
        }
        public byte ReadMotionCounterCallibri(IntPtr ptr, out uint motionCounterOut, out OpStatus outStatus)
        {
            return readMotionCounterCallibri(ptr, out motionCounterOut, out outStatus);
        }

        public byte GetSignalSettingsCallibri(IntPtr ptr, out CallibriSignalType callibriSignalTypeOut, out OpStatus outStatus)
        {
            return getSignalSettingsCallibri(ptr, out callibriSignalTypeOut, out outStatus);
        }
        public byte SetSignalSettingsCallibri(IntPtr ptr, CallibriSignalType callibriSignalType, out OpStatus outStatus)
        {
            return setSignalSettingsCallibri(ptr, callibriSignalType, out outStatus);
        }
        public byte ReadMEMSCalibrateStateCallibri(IntPtr ptr, out bool state, out OpStatus outStatus)
        {
            byte bState;
            var res = readMEMSCalibrateStateCallibri(ptr, out bState, out outStatus);
            state = bState != 0;
            return res;
        }
        public byte ReadSamplingFrequencyRespSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyRespSensor(ptr, out samplingFrequencyOut, out outStatus);
        }

        public byte AddSignalCallbackCallibri(IntPtr ptr, SignalCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackCallibri(IntPtr handle)
        {
            removeSignalCallbackCallibri(handle);
        }
        public byte AddRespirationCallbackCallibri(IntPtr ptr, RespirationCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addRespirationCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveRespirationCallbackCallibri(IntPtr handle)
        {
            removeRespirationCallbackCallibri(handle);
        }
        public byte AddElectrodeStateCallbackCallibri(IntPtr ptr, ElectrodeStateCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addElectrodeStateCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveElectrodeStateCallbackCallibri(IntPtr handle)
        {
            removeElectrodeStateCallbackCallibri(handle);
        }
        public byte AddEnvelopeDataCallbackCallibri(IntPtr ptr, EnvelopeDataCallbackCallibriSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addEnvelopeDataCallbackCallibri(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveEnvelopeDataCallbackCallibri(IntPtr handle)
        {
            removeEnvelopeDataCallbackCallibri(handle);
        }

        public byte AddQuaternionDataCallback(IntPtr ptr, QuaternionDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addQuaternionDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveQuaternionDataCallback(IntPtr handle)
        {
            removeQuaternionDataCallback(handle);
        }
        public byte AddResistCallbackBrainBit(IntPtr ptr, ResistCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit(IntPtr handle)
        {
            removeResistCallbackBrainBit(handle);
        }
        public byte AddSignalDataCallbackBrainBit(IntPtr ptr, SignalDataCallbackBrainBitSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalDataCallbackBrainBit(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalDataCallbackBrainBit(IntPtr handle)
        {
            removeSignalDataCallbackBrainBit(handle);
        }
        public byte ReadSamplingFrequencyFPGSensor(IntPtr ptr, out SensorSamplingFrequency samplingFrequencyOut, out OpStatus outStatus)
        {
            return readSamplingFrequencyFPGSensor(ptr, out samplingFrequencyOut, out outStatus);
        }
        public byte ReadIrAmplitudeFPGSensor(IntPtr ptr, out IrAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readIrAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteIrAmplitudeFPGSensor(IntPtr ptr, IrAmplitude amplitude, out OpStatus outStatus)
        {
            return writeIrAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadRedAmplitudeFPGSensor(IntPtr ptr, out RedAmplitude amplitudeOut, out OpStatus outStatus)
        {
            return readRedAmplitudeFPGSensor(ptr, out amplitudeOut, out outStatus);
        }
        public byte WriteRedAmplitudeFPGSensor(IntPtr ptr, RedAmplitude amplitude, out OpStatus outStatus)
        {
            return writeRedAmplitudeFPGSensor(ptr, amplitude, out outStatus);
        }
        public byte ReadAmpMode(IntPtr ptr, out SensorAmpMode modeOut, out OpStatus outStatus)
        {
            return readAmpMode(ptr, out modeOut, out outStatus);
        }

        public byte AddAmpModeCallback(IntPtr ptr, AmpModeCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addAmpModeCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveAmpModeCallback(IntPtr handle)
        {
            removeAmpModeCallback(handle);
        }
	    public byte PingNeuroSmart(IntPtr ptr, byte marker, out OpStatus outStatus)
        {
            return pingNeuroSmart(ptr, marker, out outStatus);
        }
        public byte AddFPGDataCallback(IntPtr ptr, FPGDataCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addFPGDataCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveFPGDataCallback(IntPtr handle)
        {
            removeFPGDataCallback(handle);
        }
	    public byte WriteSamplingFrequencySensor(IntPtr ptr, SensorSamplingFrequency samplingFrequency, out OpStatus outStatus)
        {
            return writeSamplingFrequencySensor(ptr, samplingFrequency, out outStatus);
        }


        public byte AddBatteryCallback(IntPtr ptr, BatteryCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addBatteryCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveBatteryCallback(IntPtr handle)
        {
            removeBatteryCallback(handle);
        }
        
        public byte AddConnectionStateCallback(IntPtr ptr, ConnectionStateCallbackSensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addConnectionStateCallback(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveConnectionStateCallback(IntPtr handle)
        {
            removeConnectionStateCallback(handle);
        }
                public byte AddSignalCallbackBrainBit2(IntPtr ptr, SignalCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addSignalCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveSignalCallbackBrainBit2(IntPtr handle)
        {
            removeSignalCallbackBrainBit2(handle);
        }
        public byte AddResistCallbackBrainBit2(IntPtr ptr, ResistCallbackBrainBit2Sensor callback, out IntPtr handleOut, IntPtr userData, out OpStatus outStatus)
        {
            return addResistCallbackBrainBit2(ptr, callback, out handleOut, userData, out outStatus);
        }
        public void RemoveResistCallbackBrainBit2(IntPtr handle)
        {
            removeResistCallbackBrainBit2(handle);
        }

        public byte ReadSupportedChannelsBrainBit2(IntPtr ptr, out EEGChannelInfo[] channelsOut, out OpStatus outStatus)
        {
            var cnt = getChannelsCountSensor(ptr);
            channelsOut = new EEGChannelInfo[cnt];
            var res = readSupportedChannelsBrainBit2(ptr, channelsOut, ref cnt, out outStatus);
            if (cnt != channelsOut.Length)
            {
                var chs = new EEGChannelInfo[cnt];
                Array.Copy(channelsOut, chs, cnt);
                channelsOut = chs;
            }
            return res;
        }
        public byte ReadAmplifierParamBrainBit2(IntPtr ptr, out BrainBit2AmplifierParam ampParamOut, out OpStatus outStatus)
        {
            var res = readAmplifierParamBrainBit2(ptr, out ampParamOut, out outStatus);
            if (outStatus.Success)
            {
                var cnt = getChannelsCountSensor(ptr);
                if (ampParamOut.ChSignalMode.Length != cnt)
                {
                    var chSignalMode = new BrainBit2ChannelMode[cnt];
                    Array.Copy(ampParamOut.ChSignalMode, chSignalMode, Math.Min(cnt, ampParamOut.ChSignalMode.Length));
                    ampParamOut.ChSignalMode = chSignalMode;
                }
                if (ampParamOut.ChResistUse.Length != cnt)
                {
                    var chResistUse = new bool[cnt];
                    Array.Copy(ampParamOut.ChResistUse, chResistUse, Math.Min(cnt, ampParamOut.ChResistUse.Length));
                    ampParamOut.ChResistUse = chResistUse;
                }
                if (ampParamOut.ChGain.Length != cnt)
                {
                    var chGain = new SensorGain[cnt];
                    Array.Copy(ampParamOut.ChGain, chGain, Math.Min(cnt, ampParamOut.ChGain.Length));
                    ampParamOut.ChGain = chGain;
                }
            }
            return res;
        }
        public byte WriteAmplifierParamBrainBit2(IntPtr ptr, BrainBit2AmplifierParam ampParam, out OpStatus outStatus)
        {
            var cnt = SdkLibConst.BrainBit2MaxChCount;
            if (ampParam.ChSignalMode.Length != cnt)
            {
                var chSignalMode = new BrainBit2ChannelMode[cnt];
                Array.Copy(ampParam.ChSignalMode, chSignalMode, Math.Min(cnt, ampParam.ChSignalMode.Length));
                ampParam.ChSignalMode = chSignalMode;
            }
            if (ampParam.ChResistUse.Length != cnt)
            {
                var chResistUse = new bool[cnt];
                Array.Copy(ampParam.ChResistUse, chResistUse, Math.Min(cnt, ampParam.ChResistUse.Length));
                ampParam.ChResistUse = chResistUse;
            }
            if (ampParam.ChGain.Length != cnt)
            {
                var chGain = new SensorGain[cnt];
                Array.Copy(ampParam.ChGain, chGain, Math.Min(cnt, ampParam.ChGain.Length));
                ampParam.ChGain = chGain;
            }
            return writeAmplifierParamBrainBit2(ptr, ampParam, out outStatus);
        }

    }
#endif
}

#if !ENABLE_MONO
    namespace AOT
    {
        [AttributeUsage(AttributeTargets.Method)]
        internal class MonoPInvokeCallbackAttribute : Attribute
        {
            public MonoPInvokeCallbackAttribute(Type type) { }
        }
    }
#endif
