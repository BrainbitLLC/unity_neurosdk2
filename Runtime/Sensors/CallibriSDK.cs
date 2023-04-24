using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace NeuroSDK
{
    public class CallibriSensor : Sensor
    {
        private IntPtr _this;

        private IntPtr _memsDataCallbackHandle;
        private IntPtr _quaternionDataCallbackHandle;
        private IntPtr _signalCallbackCallibriHandle;
        private IntPtr _respirationCallbackCallibriHandle;
        private IntPtr _electrodeStateCallbackCallibriHandle;
        private IntPtr _envelopeDataCallbackCallibriHandle;

        private readonly MEMSDataCallbackSensor _memsDataCallback;
        private readonly QuaternionDataCallbackSensor _quaternionDataCallback;
        private readonly SignalCallbackCallibriSensor _signalCallbackCallibri;
        private readonly RespirationCallbackCallibriSensor _respirationCallbackCallibri;
        private readonly ElectrodeStateCallbackCallibriSensor _electrodeStateCallbackCallibri;
        private readonly EnvelopeDataCallbackCallibriSensor _envelopeDataCallbackCallibri;

        public event MEMSDataRecived EventMEMSDataRecived;
        public event QuaternionDataRecived EventQuaternionDataRecived;
        public event CallibriSignalDataRecived EventCallibriSignalDataRecived;
        public event CallibriRespirationDataRecived EventCallibriRespirationDataRecived;
        public event CallibriElectrodeStateChanged EventCallibriElectrodeStateChanged;
        public event CallibriEnvelopeDataRecived EventCallibriEnvelopeDataRecived;
        internal CallibriSensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));
            OpStatus opSt;

            byte error = 0;
            _memsDataCallback = MEMSDataCallback;
            _quaternionDataCallback = QuaternionDataCallback;
            if (IsSupportedFeature(SensorFeature.FeatureMEMS))
            {
                error = SDKApiFactory.Inst.AddMEMSDataCallback(_sensorPtr, _memsDataCallback, out _memsDataCallbackHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                error = SDKApiFactory.Inst.AddQuaternionDataCallback(_sensorPtr, _quaternionDataCallback, out _quaternionDataCallbackHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }

            _signalCallbackCallibri = SignalCallbackCallibri;
            error = SDKApiFactory.Inst.AddSignalCallbackCallibri(_sensorPtr, _signalCallbackCallibri, out _signalCallbackCallibriHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _respirationCallbackCallibri = RespirationCallbackCallibri;
            if (IsSupportedFeature(SensorFeature.FeatureRespiration))
            {
                error = SDKApiFactory.Inst.AddRespirationCallbackCallibri(_sensorPtr, _respirationCallbackCallibri, out _respirationCallbackCallibriHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }

            _electrodeStateCallbackCallibri = ElectrodeStateCallbackCallibri;
            error = SDKApiFactory.Inst.AddElectrodeStateCallbackCallibri(_sensorPtr, _electrodeStateCallbackCallibri, out _electrodeStateCallbackCallibriHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _envelopeDataCallbackCallibri = EnvelopeDataCallbackCallibri;
            if (IsSupportedFeature(SensorFeature.FeatureEnvelope))
            {
                error = SDKApiFactory.Inst.AddEnvelopeDataCallbackCallibri(_sensorPtr, _envelopeDataCallbackCallibri, out _envelopeDataCallbackCallibriHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }

        private static CallibriSensor? SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as CallibriSensor;
                }
            }
            return null;
        }

        public IReadOnlyList<SensorFilter> HardwareFilters
        {
            get
            {
                SensorFilter[] arr;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadHardwareFiltersSensor(_sensorPtr, out arr, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return arr;
            }
            set
            {
                SensorFilter[] arr = value?.Count > 0 ? new SensorFilter[value.Count] : new SensorFilter[0];
                for (int i = 0; i < arr.Length; ++i) arr[i] = value[i];
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteHardwareFiltersSensor(_sensorPtr, arr, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public new SensorSamplingFrequency SamplingFrequency
        {
            get
            {
                SensorSamplingFrequency val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSamplingFrequencySensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteSamplingFrequencySensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public new SensorGain Gain
        {
            get
            {
                SensorGain val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadGainSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteGainSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public new SensorDataOffset DataOffset
        {
            get
            {
                SensorDataOffset val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadDataOffsetSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteDataOffsetSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public SensorExternalSwitchInput ExtSwInput
        {
            get
            {
                SensorExternalSwitchInput val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadExternalSwitchSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteExternalSwitchSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public new SensorFirmwareMode FirmwareMode
        {
            get
            {
                SensorFirmwareMode val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadFirmwareModeSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteFirmwareModeSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public SensorADCInput ADCInput
        {
            get
            {
                SensorADCInput val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadADCInputSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteADCInputSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }

        public SensorAccelerometerSensitivity AccSens
        {
            get
            {
                SensorAccelerometerSensitivity val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadAccelerometerSensSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteAccelerometerSensSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public SensorGyroscopeSensitivity GyroSens
        {
            get
            {
                SensorGyroscopeSensitivity val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadGyroscopeSensSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteGyroscopeSensSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public CallibriStimulatorMAState StimulatorMAStateCallibri
        {
            get
            {
                CallibriStimulatorMAState val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadStimulatorAndMAStateCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public CallibriStimulationParams StimulatorParamCallibri
        {
            get
            {
                CallibriStimulationParams val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadStimulatorParamCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteStimulatorParamCallibri(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public CallibriMotionAssistantParams MotionAssistantParamCallibri
        {
            get
            {
                CallibriMotionAssistantParams val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadMotionAssistantParamCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteMotionAssistantParamCallibri(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public CallibriMotionCounterParam MotionCounterParamCallibri
        {
            get
            {
                CallibriMotionCounterParam val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadMotionCounterParamCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteMotionCounterParamCallibri(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public uint MotionCounterCallibri
        {
            get
            {
                uint val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadMotionCounterCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public CallibriColorType ColorCallibri
        {
            get
            {
                CallibriColorType val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadColorCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public CallibriSignalType SignalTypeCallibri
        {
            get
            {
                CallibriSignalType val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.GetSignalSettingsCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.SetSignalSettingsCallibri(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public bool MEMSCalibrateStateCallibri
        {
            get
            {
                bool val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadMEMSCalibrateStateCallibri(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }

        public override void Dispose()
        {
            if (!_disposed)
            {
                SDKApiFactory.Inst.RemoveMEMSDataCallback(_memsDataCallbackHandle);
                SDKApiFactory.Inst.RemoveQuaternionDataCallback(_quaternionDataCallbackHandle);
                SDKApiFactory.Inst.RemoveSignalCallbackCallibri(_signalCallbackCallibriHandle);
                SDKApiFactory.Inst.RemoveRespirationCallbackCallibri(_respirationCallbackCallibriHandle);
                SDKApiFactory.Inst.RemoveElectrodeStateCallbackCallibri(_electrodeStateCallbackCallibriHandle);
                SDKApiFactory.Inst.RemoveEnvelopeDataCallbackCallibri(_envelopeDataCallbackCallibriHandle);
            }
            base.Dispose();
        }

        [AOT.MonoPInvokeCallback(typeof(MEMSDataCallbackSensor))]
        private static void MEMSDataCallback(IntPtr ptr, MEMSData[] dataArray, IntPtr dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventMEMSDataRecived?.Invoke(sensor, dataArray);
        }
        [AOT.MonoPInvokeCallback(typeof(QuaternionDataCallbackSensor))]
        private static void QuaternionDataCallback(IntPtr ptr, QuaternionData[] dataArray, IntPtr dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventQuaternionDataRecived?.Invoke(sensor, dataArray);
        }
        [AOT.MonoPInvokeCallback(typeof(SignalCallbackCallibriSensor))]
        private static void SignalCallbackCallibri(IntPtr ptr, CallibriSignalDataNative[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            CallibriSignalData[] rxData = new CallibriSignalData[dataArray.Length];
            for (int i = 0; i < rxData.Length; ++i)
            {
                rxData[i].PackNum = dataArray[i].PackNum;
                rxData[i].Samples = new double[dataArray[i].SzSamples];
                Marshal.Copy(dataArray[i].Samples, rxData[i].Samples, 0, rxData[i].Samples.Length);
                //Array.Copy(dataArray[i].Samples, rxData[i].Samples, rxData[i].Samples.Length);
            }
            sensor?.EventCallibriSignalDataRecived?.Invoke(sensor, rxData);
        }
        [AOT.MonoPInvokeCallback(typeof(RespirationCallbackCallibriSensor))]
        private static void RespirationCallbackCallibri(IntPtr ptr, CallibriRespirationDataNative[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            CallibriRespirationData[] rxData = new CallibriRespirationData[dataArray.Length];
            for (int i = 0; i < rxData.Length; ++i)
            {
                rxData[i].PackNum = dataArray[i].PackNum;
                rxData[i].Samples = new double[dataArray[i].SzSamples];
                Marshal.Copy(dataArray[i].Samples, rxData[i].Samples, 0, rxData[i].Samples.Length);
            }
            sensor?.EventCallibriRespirationDataRecived?.Invoke(sensor, rxData);
        }
        [AOT.MonoPInvokeCallback(typeof(ElectrodeStateCallbackCallibriSensor))]
        private static void ElectrodeStateCallbackCallibri(IntPtr ptr, CallibriElectrodeState state, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventCallibriElectrodeStateChanged?.Invoke(sensor, state);
        }
        [AOT.MonoPInvokeCallback(typeof(EnvelopeDataCallbackCallibriSensor))]
        private static void EnvelopeDataCallbackCallibri(IntPtr ptr, CallibriEnvelopeData[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventCallibriEnvelopeDataRecived?.Invoke(sensor, dataArray);
        }
    }

}
