﻿using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace NeuroSDK
{

    #region Interface impl
    public class Scanner : IScanner
    {
        private bool _disposed = false;
        private IntPtr _scannerPtr;
        private IntPtr _this;

        private readonly SensorsCallbackScanner _sensorsCallback;
        private IntPtr _sensorsCallbackHandle;

        public Scanner(params SensorFamily[] filters)
        {
            OpStatus opSt;
            _scannerPtr = SDKApiFactory.Inst.CreateScanner(filters, out opSt);
            SDKApiFactory.ThrowIfError(opSt, Convert.ToByte(opSt.Success));
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));

            _sensorsCallback = SensorsCallback;
            byte error = SDKApiFactory.Inst.AddSensorsCallbackScanner(_scannerPtr, _sensorsCallback, out _sensorsCallbackHandle, _this, out opSt); 
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        ~Scanner()
        {
            Dispose(true);
        }

        public IReadOnlyList<SensorInfo> Sensors
        {
            get
            {
                OpStatus opSt;
                SensorInfo[] sensors;
                byte error = SDKApiFactory.Inst.SensorsScanner(_scannerPtr, out sensors, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return sensors;
            }
        }

        public event SensorsChanged? EventSensorsChanged;

        public void Start()
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.StartScanner(_scannerPtr, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            
        }
        public void Stop()
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.StopScanner(_scannerPtr, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }

        public ISensor CreateSensor(SensorInfo sensorInfo)
        {
            OpStatus status;
            var sensorPtr = SDKApiFactory.Inst.CreateSensor(_scannerPtr, sensorInfo, out status);
            SDKApiFactory.ThrowIfError(status, Convert.ToByte(status.Success));
//#pragma warning disable CS8603
            if (sensorPtr == IntPtr.Zero)
                return null;

            switch (sensorInfo.SensFamily)
            {
                case SensorFamily.SensorUnknown:
                    break;
                case SensorFamily.SensorLECallibri:
                case SensorFamily.SensorLEKolibri:
                    return new CallibriSensor(sensorPtr);
                case SensorFamily.SensorLEBrainBit:
                    return new BrainBitSensor(sensorPtr);
                case SensorFamily.SensorLEBrainBitBlack:
                    return new BrainBitBlackSensor(sensorPtr);

                default:
                    return new Sensor(sensorPtr);
            }
            return new Sensor(sensorPtr);
            //#pragma warning restore CS8603
        }

        #region Internal
        [AOT.MonoPInvokeCallback(typeof(SensorsCallbackScanner))]
        private static void SensorsCallback(IntPtr ptr, IntPtr sensorsPtr, IntPtr szSensors, IntPtr userData)
        {
            var scanner = SDKScanner(userData);
            var sensors = new NativeArrayMarshaler<SensorInfo>().MarshalArray(sensorsPtr, szSensors);
            scanner?.EventSensorsChanged?.Invoke(scanner, sensors);
        }

        private static Scanner SDKScanner(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as Scanner;
                }
            }
            return null;
        }
        #endregion
        #region Dispose
        /// <summary>
        /// <inheritdoc/>
        /// </summary>
        public void Dispose()
        {
            // Dispose of unmanaged resources.
            Dispose(true);
            // Suppress finalization.
            GC.SuppressFinalize(this);
        }
        private void Dispose(bool disposing)
        {
            if (_disposed)
                return;
            _disposed = true;
            try
            {
                SDKApiFactory.Inst.RemoveSensorsCallbackScanner(_sensorsCallbackHandle);
                _sensorsCallbackHandle = IntPtr.Zero;
                SDKApiFactory.Inst.FreeScanner(_scannerPtr);
                _scannerPtr = IntPtr.Zero;
                var handle = GCHandle.FromIntPtr(_this);
                if (handle.IsAllocated)
                {
                    handle.Free();
                    _this = IntPtr.Zero;
                }
            }
            catch (Exception)
            {
            }
        }
        #endregion
    }
    public class Sensor : ISensor
    {
        protected bool _disposed = false;
        protected IntPtr _sensorPtr;
        private IntPtr _this;

        private IntPtr _batteryCallbackHandle;
        private IntPtr _connectionStateCallbackHandle;

        private readonly BatteryCallbackSensor _batteryCallback;
        private readonly ConnectionStateCallbackSensor _connectionStateCallback;

        internal Sensor(IntPtr sensorPtr)
        {
            _sensorPtr = sensorPtr;
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));

            OpStatus opSt;
            byte error = 0;
            _batteryCallback = BatteryCallback; 
            if (IsSupportedParameter(SensorParameter.ParameterBattPower))
            {
                error = SDKApiFactory.Inst.AddBatteryCallback(_sensorPtr, _batteryCallback, out _batteryCallbackHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }

            _connectionStateCallback = ConnectionStateCallback;
            error = SDKApiFactory.Inst.AddConnectionStateCallback(_sensorPtr, _connectionStateCallback, out _connectionStateCallbackHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

        }
        ~Sensor()
        {
            Dispose(true);
        }

        public SensorFamily SensFamily { get => SDKApiFactory.Inst.GetFamilySensor(_sensorPtr); }
        public IReadOnlyList<SensorFeature> Features
        {
            get
            {
                var sz = SDKApiFactory.Inst.GetFeaturesCountSensor(_sensorPtr);
                SensorFeature[] arr = new SensorFeature[sz];
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.GetFeaturesSensor(_sensorPtr, arr, ref sz, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return arr;
            }
        }
        public IReadOnlyList<SensorCommand> Commands
        {
            get
            {
                var sz = SDKApiFactory.Inst.GetCommandsCountSensor(_sensorPtr);
                SensorCommand[] arr = new SensorCommand[sz];
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.GetCommandsSensor(_sensorPtr, arr, ref sz, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return arr;
            }
        }
        public IReadOnlyList<ParameterInfo> Parameters
        {
            get
            {
                var sz = SDKApiFactory.Inst.GetParametersCountSensor(_sensorPtr);
                ParameterInfo[] arr = new ParameterInfo[sz];
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.GetParametersSensor(_sensorPtr, arr, ref sz, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return arr;
            }
        }
        public string Name
        {
            get
            {
                string val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadNameSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteNameSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public SensorState State
        {
            get
            {
                SensorState val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadStateSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public string Address
        {
            get
            {
                string val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadAddressSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public string SerialNumber
        {
            get
            {
                string val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSerialNumberSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteSerialNumberSensor(_sensorPtr, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public int BattPower
        {
            get
            {
                int val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadBattPowerSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public SensorVersion Version
        {
            get
            {
                SensorVersion val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadVersionSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }

        public SensorSamplingFrequency SamplingFrequency
        {
            get
            {
                SensorSamplingFrequency val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSamplingFrequencySensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        
        public SensorGain Gain
        {
            get
            {
                SensorGain val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadGainSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }

        public SensorDataOffset DataOffset
        {
            get
            {
                SensorDataOffset val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadDataOffsetSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }
        public SensorFirmwareMode FirmwareMode
        {
            get
            {
                SensorFirmwareMode val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadFirmwareModeSensor(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }


        public event BatteryChanged? EventBatteryChanged;
        public event SensorStateChanged? EventSensorStateChanged;


        public void Connect()
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.ConnectSensor(_sensorPtr, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        public void Disconnect()
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.DisconnectSensor(_sensorPtr, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        public bool IsSupportedFeature(SensorFeature sensorFeature)
        {
            return SDKApiFactory.Inst.IsSupportedFeatureSensor(_sensorPtr, sensorFeature) == 1;
        }
        public bool IsSupportedCommand(SensorCommand sensorCommand)
        {
            return SDKApiFactory.Inst.IsSupportedCommandSensor(_sensorPtr, sensorCommand) == 1;
        }
        public bool IsSupportedParameter(SensorParameter sensorParameter)
        {
            return SDKApiFactory.Inst.IsSupportedParameterSensor(_sensorPtr, sensorParameter) == 1;
        }
        public void ExecCommand(SensorCommand sensorCommand)
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.ExecCommandSensor(_sensorPtr, sensorCommand, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        #region Internal

        [AOT.MonoPInvokeCallback(typeof(BatteryCallbackSensor))]
        private static void BatteryCallback(IntPtr ptr, int battPower, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventBatteryChanged?.Invoke(sensor, battPower);
        }
        [AOT.MonoPInvokeCallback(typeof(ConnectionStateCallbackSensor))]
        private static void ConnectionStateCallback(IntPtr ptr, SensorState state, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventSensorStateChanged?.Invoke(sensor, state);
        }

        private static Sensor SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as Sensor;
                }
            }
            return null;
        }
        #endregion
        #region Dispose
        /// <summary>
        /// <inheritdoc/>
        /// </summary>
        public virtual void Dispose()
        {
            // Dispose of unmanaged resources.
            Dispose(true);
            // Suppress finalization.
            GC.SuppressFinalize(this);
        }
        private void Dispose(bool disposing)
        {
            if (_disposed)
                return;
            _disposed = true;
            try
            {
                SDKApiFactory.Inst.RemoveBatteryCallback(_batteryCallbackHandle);
                SDKApiFactory.Inst.RemoveConnectionStateCallback(_connectionStateCallbackHandle);

                SDKApiFactory.Inst.FreeSensor(_sensorPtr);

                _sensorPtr = IntPtr.Zero;
                var handle = GCHandle.FromIntPtr(_this);
                if (handle.IsAllocated)
                {
                    handle.Free();
                    _this = IntPtr.Zero;
                }
            }
            catch (Exception)
            {
            }
        }
        #endregion
    }
    #endregion

}
