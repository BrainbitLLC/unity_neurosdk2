using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace NeuroSDK
{
    public class BrainBitBlackSensor : BrainBitSensor
    {
        private IntPtr _this;

        private IntPtr _fpgDataCallbackNeuroSmartHandle;
        private IntPtr _modeCallbackNeuroSmartHandle;
        private IntPtr _memsDataCallbackHandle;

        private readonly FPGDataCallbackNeuroSmartSensor _fpgDataCallbackNeuroSmart;
        private readonly AmpModeCallbackSensor _modeCallbackNeuroSmart;
        private readonly MEMSDataCallbackSensor _memsDataCallback;

        public event FPGDataRecived? EventFPGDataRecived;
        public event SensorAmpModeChanged? EventSensorAmpModeChanged;
        public event MEMSDataRecived? EventMEMSDataRecived;

        internal BrainBitBlackSensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));
            OpStatus opSt;

            _modeCallbackNeuroSmart = AmpModeCallbackSensor;
            byte error = SDKApiFactory.Inst.AddAmpModeCallback(_sensorPtr, _modeCallbackNeuroSmart, out _modeCallbackNeuroSmartHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _fpgDataCallbackNeuroSmart = FPGDataCallbackNeuroSmart;
            if (IsSupportedFeature(SensorFeature.FeatureFPG))
            {
                error = SDKApiFactory.Inst.AddFPGDataCallbackNeuroSmart(_sensorPtr, _fpgDataCallbackNeuroSmart, out _fpgDataCallbackNeuroSmartHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
            _memsDataCallback = MEMSDataCallback;
            if (IsSupportedFeature(SensorFeature.FeatureMEMS))
            {
                error = SDKApiFactory.Inst.AddMEMSDataCallback(_sensorPtr, _memsDataCallback, out _memsDataCallbackHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }

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
        public IrAmplitude IrAmplitudeHeadband
        {
            get
            {
                IrAmplitude val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadIrAmplitudeHeadband(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteIrAmplitudeHeadband(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public RedAmplitude RedAmplitudeHeadband
        {
            get
            {
                RedAmplitude val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadRedAmplitudeHeadband(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteRedAmplitudeHeadband(_sensorPtr, value, out opSt);
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
                SDKApiFactory.Inst.RemoveFPGDataCallbackNeuroSmart(_fpgDataCallbackNeuroSmartHandle);
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
        [AOT.MonoPInvokeCallback(typeof(FPGDataCallbackNeuroSmartSensor))]
        private static void FPGDataCallbackNeuroSmart(IntPtr ptr, FPGData[] dataArray, int dataSize, IntPtr userData)
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
