using System;
using System.Runtime.InteropServices;

namespace NeuroSDK
{
    public class Headphones2Sensor : Sensor
    {
        private IntPtr _this;

        private IntPtr _fpgDataCallbackHandle;
        private IntPtr _memsDataCallbackHandle;
        private IntPtr _modeCallbackNeuroSmartHandle;
        private IntPtr _resistCallbackHeadphones2Handle;
        private IntPtr _signalDataCallbackHeadphones2Handle;

        private readonly FPGDataCallbackSensor _fpgDataCallback;
        private readonly MEMSDataCallbackSensor _memsDataCallback;
        private readonly AmpModeCallbackSensor _modeCallbackNeuroSmart;
        private readonly ResistCallbackHeadphones2Sensor _resistCallbackHeadphones2;
        private readonly SignalDataCallbackHeadphones2Sensor _signalDataCallbackHeadphones2;

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        public event FPGDataRecived? EventFPGDataRecived;
        public event MEMSDataRecived? EventMEMSDataRecived;
        public event SensorAmpModeChanged? EventSensorAmpModeChanged;
        public event Headphones2ResistDataRecived? EventHeadphones2ResistDataRecived;
        public event Headphones2SignalDataRecived? EventHeadphones2SignalDataRecived;
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        internal Headphones2Sensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));
            OpStatus opSt;
            byte error = 0;

            _modeCallbackNeuroSmart = AmpModeCallbackSensor;
            error = SDKApiFactory.Inst.AddAmpModeCallback(_sensorPtr, _modeCallbackNeuroSmart, out _modeCallbackNeuroSmartHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _resistCallbackHeadphones2 = ResistCallbackHeadphones2;
            error = SDKApiFactory.Inst.AddResistCallbackHeadphones2(_sensorPtr, _resistCallbackHeadphones2, out _resistCallbackHeadphones2Handle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _signalDataCallbackHeadphones2 = SignalDataCallbackHeadphones2;
            error = SDKApiFactory.Inst.AddSignalDataCallbackHeadphones2(_sensorPtr, _signalDataCallbackHeadphones2, out _signalDataCallbackHeadphones2Handle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _fpgDataCallback = FPGDataCallback;
            if (IsSupportedFeature(SensorFeature.FeatureFPG))
            {
                error = SDKApiFactory.Inst.AddFPGDataCallback(_sensorPtr, _fpgDataCallback, out _fpgDataCallbackHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
            _memsDataCallback = MEMSDataCallback;
            if (IsSupportedFeature(SensorFeature.FeatureMEMS))
            {
                error = SDKApiFactory.Inst.AddMEMSDataCallback(_sensorPtr, _memsDataCallback, out _memsDataCallbackHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }

        public SensorSamplingFrequency SamplingFrequencyMEMS
        {
            get
            {
                SensorSamplingFrequency val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSamplingFrequencyMEMSSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public SensorSamplingFrequency SamplingFrequencyFPG
        {
            get
            {
                SensorSamplingFrequency val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSamplingFrequencyFPGSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public SensorSamplingFrequency SamplingFrequencyResist
        {
            get
            {
                SensorSamplingFrequency val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSamplingFrequencyResistSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public Headphones2AmplifierParam AmplifierParamHeadphones2
        {
            get
            {
                Headphones2AmplifierParam val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadAmplifierParamHeadphones2(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteAmplifierParamHeadphones2(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public SensorAmpMode AmpMode
        {
            get
            {
                SensorAmpMode val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadAmpMode(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
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
        public IrAmplitude IrAmplitudeFPGSensor
        {
            get
            {
                IrAmplitude val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadIrAmplitudeFPGSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteIrAmplitudeFPGSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public RedAmplitude RedAmplitudeFPGSensor
        {
            get
            {
                RedAmplitude val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadRedAmplitudeFPGSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteRedAmplitudeFPGSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        private static Headphones2Sensor? SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as Headphones2Sensor;
                }
            }
            return null;
        }
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        public void PingNeuroSmart(byte marker)
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.PingNeuroSmart(_sensorPtr, marker, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }

        public override void Dispose()
        {
            if (!_disposed)
            {
                SDKApiFactory.Inst.RemoveSignalDataCallbackHeadphones2(_signalDataCallbackHeadphones2Handle);
                SDKApiFactory.Inst.RemoveResistCallbackHeadphones2(_resistCallbackHeadphones2Handle);
                SDKApiFactory.Inst.RemoveAmpModeCallback(_modeCallbackNeuroSmartHandle);
                SDKApiFactory.Inst.RemoveFPGDataCallback(_fpgDataCallbackHandle);
                SDKApiFactory.Inst.RemoveMEMSDataCallback(_memsDataCallbackHandle);
            }
            base.Dispose();
        }

        [AOT.MonoPInvokeCallback(typeof(AmpModeCallbackSensor))]
        private static void AmpModeCallbackSensor(IntPtr ptr, SensorAmpMode mode, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventSensorAmpModeChanged?.Invoke(sensor, mode);
        }
        [AOT.MonoPInvokeCallback(typeof(ResistCallbackHeadphones2Sensor))]
        private static void ResistCallbackHeadphones2(IntPtr ptr, Headphones2ResistData[] data, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventHeadphones2ResistDataRecived?.Invoke(sensor, data);
        }
        [AOT.MonoPInvokeCallback(typeof(SignalDataCallbackHeadphones2Sensor))]
        private static void SignalDataCallbackHeadphones2(IntPtr ptr, Headphones2SignalData[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventHeadphones2SignalDataRecived?.Invoke(sensor, dataArray);
        }
        [AOT.MonoPInvokeCallback(typeof(FPGDataCallbackSensor))]
        private static void FPGDataCallback(IntPtr ptr, FPGData[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventFPGDataRecived?.Invoke(sensor, dataArray);
        }
        [AOT.MonoPInvokeCallback(typeof(MEMSDataCallbackSensor))]
        private static void MEMSDataCallback(IntPtr ptr, MEMSData[] dataArray, IntPtr dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventMEMSDataRecived?.Invoke(sensor, dataArray);
        }
    }
}
