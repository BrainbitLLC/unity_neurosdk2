using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace NeuroSDK
{
    public class BrainBitBlackSensor : BrainBitSensor
    {
        private IntPtr _this;

        private IntPtr _fpgDataCallbackHandle;
        private IntPtr _modeCallbackNeuroSmartHandle;
        private IntPtr _memsDataCallbackHandle;

        private readonly FPGDataCallbackSensor _fpgDataCallback;
        private readonly AmpModeCallbackSensor _modeCallbackNeuroSmart;
        private readonly MEMSDataCallbackSensor _memsDataCallback;

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        public event FPGDataRecived? EventFPGDataRecived;
        public event SensorAmpModeChanged? EventSensorAmpModeChanged;
        public event MEMSDataRecived? EventMEMSDataRecived;
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        internal BrainBitBlackSensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));
            OpStatus opSt;

            _modeCallbackNeuroSmart = AmpModeCallbackSensor;
            byte error = SDKApiFactory.Inst.AddAmpModeCallback(_sensorPtr, _modeCallbackNeuroSmart, out _modeCallbackNeuroSmartHandle, _this, out opSt);
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

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        private static BrainBitBlackSensor? SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as BrainBitBlackSensor;
                }
            }
            return null;
        }
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

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
        #region For models with numbers 3+
        public IReadOnlyList<EEGChannelInfo> SupportedChannelsBrainBit2
        {
            get
            {
                EEGChannelInfo[] val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSupportedChannelsBrainBit2(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public BrainBit2AmplifierParam AmplifierParamBrainBit2
        {
            get
            {
                BrainBit2AmplifierParam val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadAmplifierParamBrainBit2(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteAmplifierParamBrainBit2(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        #endregion

        public void PingNeuroSmart(byte marker)
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.PingNeuroSmart(_sensorPtr, marker, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }

        public override void Dispose()
        {
            if(!_disposed)
            {
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
