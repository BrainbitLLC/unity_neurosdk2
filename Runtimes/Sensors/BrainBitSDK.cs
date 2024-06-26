using System;
using System.Runtime.InteropServices;

namespace NeuroSDK
{
    public class BrainBitSensor : Sensor
    {
        private IntPtr _this;

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        public event BrainBitResistDataRecived? EventBrainBitResistDataRecived;
        public event BrainBitSignalDataRecived? EventBrainBitSignalDataRecived;
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        private IntPtr _resistCallbackBrainBitHandle;
        private IntPtr _signalDataCallbackBrainBitHandle;

        private readonly ResistCallbackBrainBitSensor _resistCallbackBrainBit;
        private readonly SignalDataCallbackBrainBitSensor _signalDataCallbackBrainBit;

        internal BrainBitSensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));

            _resistCallbackBrainBit = ResistCallbackBrainBit;
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.AddResistCallbackBrainBit(_sensorPtr, _resistCallbackBrainBit, out _resistCallbackBrainBitHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _signalDataCallbackBrainBit = SignalDataCallbackBrainBit;
            error = SDKApiFactory.Inst.AddSignalDataCallbackBrainBit(_sensorPtr, _signalDataCallbackBrainBit, out _signalDataCallbackBrainBitHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        private static BrainBitSensor? SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as BrainBitSensor;
                }
            }
            return null;
        }
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

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

        public override void Dispose()
        {
            if (!_disposed)
            {
                SDKApiFactory.Inst.RemoveResistCallbackBrainBit(_resistCallbackBrainBitHandle);
                SDKApiFactory.Inst.RemoveSignalDataCallbackBrainBit(_signalDataCallbackBrainBitHandle);
            }
            base.Dispose();
        }

        [AOT.MonoPInvokeCallback(typeof(ResistCallbackBrainBitSensor))]
        private static void ResistCallbackBrainBit(IntPtr ptr, BrainBitResistData data, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventBrainBitResistDataRecived?.Invoke(sensor, data);
        }
        [AOT.MonoPInvokeCallback(typeof(SignalDataCallbackBrainBitSensor))]
        private static void SignalDataCallbackBrainBit(IntPtr ptr, BrainBitSignalData[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventBrainBitSignalDataRecived?.Invoke(sensor, dataArray);
        }
    }
}