using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace NeuroSDK
{
    public class PhotoStimSensor : Sensor
    {
        private IntPtr _this;

        private IntPtr _modeCallbackPhotoStimHandle;
        private IntPtr _syncStateCallbackPhotoStimHandle;
        private readonly StimulModeCallbackSensor _modeCallbackPhotoStim;
        private readonly StimulSyncStateCallbackSensor _syncStateCallbackPhotoStim;

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        public event SensorStimulModeChanged? EventSensorStimModeChanged;
        public event SensorStimulSyncStateChanged? EventSensorPhotoStimSyncStateChanged;
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        internal PhotoStimSensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));

            OpStatus opSt;
            byte error = 0;

            _modeCallbackPhotoStim = StimModeCallbackSensor;
            error = SDKApiFactory.Inst.AddStimModeCallback(_sensorPtr, _modeCallbackPhotoStim, out _modeCallbackPhotoStimHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            _syncStateCallbackPhotoStim = PhotoStimSyncStateCallbackSensor;
            error = SDKApiFactory.Inst.AddPhotoStimSyncStateCallback(_sensorPtr, _syncStateCallbackPhotoStim, out _syncStateCallbackPhotoStimHandle, _this, out opSt);
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
        public SensorStimulSyncState PhotoStimSyncState
        {
            get
            {
                SensorStimulSyncState val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadPhotoStimSyncState(_sensorPtr, out val, out opSt);
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
        public double PhotoStimTimeDefer
        {
            get
            {
                double val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadPhotoStimTimeDefer(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WritePhotoStimTimeDefer(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }


        public override void Dispose()
        {
            if (!_disposed)
            {
                SDKApiFactory.Inst.RemoveStimModeCallback(_modeCallbackPhotoStimHandle);
                SDKApiFactory.Inst.RemovePhotoStimSyncStateCallback(_syncStateCallbackPhotoStimHandle);
            }
            base.Dispose();
        }

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        private static PhotoStimSensor? SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as PhotoStimSensor;
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
        private static void PhotoStimSyncStateCallbackSensor(IntPtr ptr, SensorStimulSyncState state, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventSensorPhotoStimSyncStateChanged?.Invoke(sensor, state);
        }
    }
}