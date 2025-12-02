using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace NeuroSDK
{
    public class NeuroStimSensor : Sensor
    {
        private IntPtr _this;

        private IntPtr _modeCallbackNeuroStimHandle;
        private IntPtr _syncStateCallbackNeuroStimHandle;
        private IntPtr _batteryGaugeStateCallbackSensorHandle;
        private readonly StimulModeCallbackSensor _modeCallbackNeuroStim;
        private readonly StimulSyncStateCallbackSensor _syncStateCallbackNeuroStim;
        private readonly BatteryGaugeStateCallbackSensor _batteryGaugeStateCallbackSensor;

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        public event SensorStimulModeChanged? EventSensorStimModeChanged;
        public event SensorStimulSyncStateChanged? EventSensorNeuroStimSyncStateChanged;
        public event BatteryGaugeStateChanged? EventBatteryGaugeStateChanged;
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        internal NeuroStimSensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));

            OpStatus opSt;
            byte error = 0;

            _modeCallbackNeuroStim = StimModeCallbackSensor;
            error = SDKApiFactory.Inst.AddStimModeCallback(_sensorPtr, _modeCallbackNeuroStim, out _modeCallbackNeuroStimHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            _syncStateCallbackNeuroStim = NeuroStimSyncStateCallbackSensor;
            error = SDKApiFactory.Inst.AddNeuroStimSyncStateCallback(_sensorPtr, _syncStateCallbackNeuroStim, out _syncStateCallbackNeuroStimHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _batteryGaugeStateCallbackSensor = BatteryGaugeStateCallbackSensor;
            error = SDKApiFactory.Inst.AddBatteryGaugeStateCallback(_sensorPtr, _batteryGaugeStateCallbackSensor, out _batteryGaugeStateCallbackSensorHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }

        public SensorStimulMode StimMode
        {
            get
            {
                SensorStimulMode val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadStimMode(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public SensorStimulSyncState NeuroStimSyncState
        {
            get
            {
                SensorStimulSyncState val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadNeuroStimSyncState(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public IReadOnlyList<StimulPhase> StimPrograms
        {
            get
            {
                StimulPhase[] val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadStimPrograms(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                StimulPhase[] arr = value?.Count > 0 ? new StimulPhase[value.Count] : new StimulPhase[0];
                for (int i = 0; i < arr.Length; ++i) arr[i] = value![i];
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteStimPrograms(_sensorPtr, arr, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public double NeuroStimTimeDefer
        {
            get
            {
                double val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadNeuroStimTimeDefer(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteNeuroStimTimeDefer(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public SensorStimulType CalibrateSignalType
        {
            get
            {
                SensorStimulType val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadCalibrateSignalType(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteCalibrateSignalType(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public SensorBatteryGauge BatteryGauge
        {
            get
            {
                SensorBatteryGauge val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadBatteryGaugeState(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }


        public override void Dispose()
        {
            if (!_disposed)
            {
                SDKApiFactory.Inst.RemoveStimModeCallback(_modeCallbackNeuroStimHandle);
                SDKApiFactory.Inst.RemoveNeuroStimSyncStateCallback(_syncStateCallbackNeuroStimHandle);
                SDKApiFactory.Inst.RemoveBatteryGaugeStateCallback(_batteryGaugeStateCallbackSensorHandle);
            }
            base.Dispose();
        }

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        private static NeuroStimSensor? SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as NeuroStimSensor;
                }
            }
            return null;
        }
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        [AOT.MonoPInvokeCallback(typeof(StimulModeCallbackSensor))]
        private static void StimModeCallbackSensor(IntPtr ptr, SensorStimulMode mode, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventSensorStimModeChanged?.Invoke(sensor, mode);
        }
        [AOT.MonoPInvokeCallback(typeof(StimulSyncStateCallbackSensor))]
        private static void NeuroStimSyncStateCallbackSensor(IntPtr ptr, SensorStimulSyncState state, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventSensorNeuroStimSyncStateChanged?.Invoke(sensor, state);
        }
        [AOT.MonoPInvokeCallback(typeof(BatteryGaugeStateCallbackSensor))]
        private static void BatteryGaugeStateCallbackSensor(IntPtr ptr, SensorBatteryGauge battGauge, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventBatteryGaugeStateChanged?.Invoke(sensor, battGauge);
        }
    }
}