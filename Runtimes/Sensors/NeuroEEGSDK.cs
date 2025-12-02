using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace NeuroSDK
{
    public class NeuroEEGSensor : Sensor
    {
        private IntPtr _this;

        private IntPtr _modeCallbackNeuroSmartHandle;
        private IntPtr _modeCallbackNeuroStimHandle;
        private IntPtr _batteryGaugeStateCallbackSensorHandle;
        private IntPtr _syncStateCallbackNeuroStimHandle;
        private IntPtr _signalCallbackNeuroEEGSensorHandle;
        private IntPtr _resistCallbackNeuroEEGSensorHandle;
        private IntPtr _signalResistCallbackNeuroEEGSensorHandle;
        private IntPtr _signalRawCallbackNeuroEEGSensorHandle;
        private IntPtr _fileStreamReadCallbackNeuroEEGSensorHandle;

        private readonly AmpModeCallbackSensor _modeCallbackNeuroSmart;
        private readonly StimulModeCallbackSensor _modeCallbackNeuroStim;
        private readonly BatteryGaugeStateCallbackSensor _batteryGaugeStateCallbackSensor;
        private readonly StimulSyncStateCallbackSensor _syncStateCallbackNeuroStim;
        private readonly SignalCallbackNeuroEEGSensor _signalCallbackNeuroEEGSensor;
        private readonly ResistCallbackNeuroEEGSensor _resistCallbackNeuroEEGSensor;
        private readonly SignalResistCallbackNeuroEEGSensor _signalResistCallbackNeuroEEGSensor;
        private readonly SignalRawCallbackNeuroEEGSensor _signalRawCallbackNeuroEEGSensor;
        private readonly FileStreamReadCallbackNeuroEEGSensor _fileStreamReadCallbackNeuroEEGSensor;


#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        private NeuroStimSensor? _moduleStim;

        public event SensorAmpModeChanged? EventSensorAmpModeChanged;
        public event SensorStimulModeChanged? EventSensorStimModeChanged;
        public event BatteryGaugeStateChanged? EventBatteryGaugeStateChanged;
        public event SensorStimulSyncStateChanged? EventSensorNeuroStimSyncStateChanged;
        public event NeuroEEGSignalDataRecived? EventNeuroEEGSignalDataRecived;
        public event NeuroEEGResistDataRecived? EventNeuroEEGResistDataRecived;
        public event NeuroEEGSignalResistDataRecived? EventNeuroEEGSignalResistDataRecived;
        public event NeuroEEGSignalRawDataRecived? EventNeuroEEGSignalRawDataRecived;
        public event NeuroEEGFileStreamReadRecived? EventNeuroEEGFileStreamReadRecived;
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        internal NeuroEEGSensor(IntPtr sensorPtr) : base(sensorPtr)
        {
            _this = GCHandle.ToIntPtr(GCHandle.Alloc(this));

            OpStatus opSt;
            byte error = 0;

            _modeCallbackNeuroSmart = AmpModeCallbackSensor;
            error = SDKApiFactory.Inst.AddAmpModeCallback(_sensorPtr, _modeCallbackNeuroSmart, out _modeCallbackNeuroSmartHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _signalCallbackNeuroEEGSensor = SignalCallbackNeuroEEGSensor;
            error = SDKApiFactory.Inst.AddSignalCallbackNeuroEEG(_sensorPtr, _signalCallbackNeuroEEGSensor, out _signalCallbackNeuroEEGSensorHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _resistCallbackNeuroEEGSensor = ResistCallbackNeuroEEGSensor;
            error = SDKApiFactory.Inst.AddResistCallbackNeuroEEG(_sensorPtr, _resistCallbackNeuroEEGSensor, out _resistCallbackNeuroEEGSensorHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _signalResistCallbackNeuroEEGSensor = SignalResistCallbackNeuroEEGSensor;
            error = SDKApiFactory.Inst.AddSignalResistCallbackNeuroEEG(_sensorPtr, _signalResistCallbackNeuroEEGSensor, out _signalResistCallbackNeuroEEGSensorHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _signalRawCallbackNeuroEEGSensor = SignalRawCallbackNeuroEEGSensor;
            error = SDKApiFactory.Inst.AddSignalRawCallbackNeuroEEG(_sensorPtr, _signalRawCallbackNeuroEEGSensor, out _signalRawCallbackNeuroEEGSensorHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _fileStreamReadCallbackNeuroEEGSensor = FileStreamReadCallbackNeuroEEGSensor;
            error = SDKApiFactory.Inst.AddFileStreamReadCallbackNeuroEEG(_sensorPtr, _fileStreamReadCallbackNeuroEEGSensor, out _fileStreamReadCallbackNeuroEEGSensorHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);

            _modeCallbackNeuroStim = StimModeCallbackSensor;
            _syncStateCallbackNeuroStim = NeuroStimSyncStateCallbackSensor;
            if (IsSupportedFeature(SensorFeature.FeaturePhotoStimulator) || IsSupportedFeature(SensorFeature.FeatureAcousticStimulator))
            {
                error = SDKApiFactory.Inst.AddStimModeCallback(_sensorPtr, _modeCallbackNeuroStim, out _modeCallbackNeuroStimHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);

                error = SDKApiFactory.Inst.AddNeuroStimSyncStateCallback(_sensorPtr, _syncStateCallbackNeuroStim, out _syncStateCallbackNeuroStimHandle, _this, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }

            _batteryGaugeStateCallbackSensor = BatteryGaugeStateCallbackSensor;
            error = SDKApiFactory.Inst.AddBatteryGaugeStateCallback(_sensorPtr, _batteryGaugeStateCallbackSensor, out _batteryGaugeStateCallbackSensorHandle, _this, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }

        public uint SurveyId
        {
            get
            {
                uint val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSurveyIdNeuroEEG(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteSurveyIdNeuroEEG(_sensorPtr, value, out opSt);
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

        public IReadOnlyList<EEGChannelInfo> SupportedChannelsNeuroEEG
        {
            get
            {
                EEGChannelInfo[] val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadSupportedChannelsNeuroEEG(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }

        public NeuroEEGFSStatus FSStatus
        {
            get
            {
                NeuroEEGFSStatus val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadFilesystemStatusNeuroEEG(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }

        public SensorDiskInfo FSDiskInfo
        {
            get
            {
                SensorDiskInfo val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadFileSystemDiskInfoNeuroEEG(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return val;
            }
        }

        public NeuroEEGAmplifierParam AmplifierParamNeuroEEG
        {
            get
            {
                NeuroEEGAmplifierParamNative val;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadAmplifierParamNeuroEEG(_sensorPtr, out val, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                NeuroEEGAmplifierParam res = new()
                {
                    ReferentResistMesureAllow = val.ReferentResistMesureAllow == 1,
                    Frequency = val.Frequency,
                    ReferentMode = val.ReferentMode,
                    ChannelMode = new EEGChannelMode[val.ChannelMode.Length],
                    ChannelGain = new SensorGain[val.ChannelGain.Length],
                    UseDiffAsRespiration = val.UseDiffAsRespiration == 1,
                };
                val.ChannelMode.CopyTo(res.ChannelMode, 0);
                val.ChannelGain.CopyTo(res.ChannelGain, 0);
                return res;
            }
            set
            {
                OpStatus opSt;
                NeuroEEGAmplifierParamNative setter = new()
                {
                    ReferentResistMesureAllow = value.ReferentResistMesureAllow ? (byte)1 : (byte)0,
                    Frequency = value.Frequency,
                    ReferentMode = value.ReferentMode,
                    ChannelMode = new EEGChannelMode[value.ChannelMode.Length],
                    ChannelGain = new SensorGain[value.ChannelGain.Length],
                    UseDiffAsRespiration = value.UseDiffAsRespiration ? (byte)1 : (byte)0,
                };
                value.ChannelMode.CopyTo(setter.ChannelMode, 0);
                value.ChannelGain.CopyTo(setter.ChannelGain, 0);
                byte error = SDKApiFactory.Inst.WriteAmplifierParamNeuroEEG(_sensorPtr, setter, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
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
#pragma warning disable CS8632
        public NeuroStimSensor? ModuleStim
        {
            get => _moduleStim;
            set
            {
                if (_moduleStim == value) return;
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.WriteModuleStimNeuroEEG(_sensorPtr, value == null ? IntPtr.Zero : value._sensorPtr, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                _moduleStim = value;
            }
        }
#pragma warning restore CS8632
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
        public double MinBrightnessPrcLed
        {
            get
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadLedStateNeuroEEG(_sensorPtr, out var minBrightnessPrcOut, out var maxBrightnessPrcOut, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return minBrightnessPrcOut;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadLedStateNeuroEEG(_sensorPtr, out var minBrightnessPrcOut, out var maxBrightnessPrcOut, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                error = SDKApiFactory.Inst.WriteLedStateNeuroEEG(_sensorPtr, value, maxBrightnessPrcOut, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }
        public double MaxBrightnessPrcLed
        {
            get
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadLedStateNeuroEEG(_sensorPtr, out var minBrightnessPrcOut, out var maxBrightnessPrcOut, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                return maxBrightnessPrcOut;
            }
            set
            {
                OpStatus opSt;
                byte error = SDKApiFactory.Inst.ReadLedStateNeuroEEG(_sensorPtr, out var minBrightnessPrcOut, out var maxBrightnessPrcOut, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
                error = SDKApiFactory.Inst.WriteLedStateNeuroEEG(_sensorPtr, minBrightnessPrcOut, value, out opSt);
                SDKApiFactory.ThrowIfError(opSt, error);
            }
        }

        public SensorFileInfo ReadFileInfoNeuroEEG(string fileName)
        {
            SensorFileInfo val;
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.ReadFileInfoNeuroEEG(_sensorPtr, fileName, out val, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            return val;
        }
        public SensorFileInfo[] ReadFileInfoAllNeuroEEG(uint maxFiles = 0xFFFF)
        {
            SensorFileInfo[] val;
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.ReadFileInfoAllNeuroEEG(_sensorPtr, out val, maxFiles, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            return val;
        }
        public void WriteFileNeuroEEG(string fileName, byte[] data, uint offsetStart = 0)
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.WriteFileNeuroEEG(_sensorPtr, fileName, data, offsetStart, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        public byte[] ReadFileNeuroEEG(string fileName, uint szData = 0xFFFFFFFF, uint offsetStart = 0)
        {
            if (szData == 0xFFFFFFFF)
            {
                var fInfo = ReadFileInfoNeuroEEG(fileName);
                if (offsetStart >= fInfo.FileSize)
                    return new byte[0];
                szData = fInfo.FileSize - offsetStart;
            }
            OpStatus opSt;
            byte[] val;
            byte error = SDKApiFactory.Inst.ReadFileNeuroEEG(_sensorPtr, fileName, out val, szData, offsetStart, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            return val;
        }
        public void DeleteFileNeuroEEG(string fileName)
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.DeleteFileNeuroEEG(_sensorPtr, fileName, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        public void DeleteAllFilesNeuroEEG(string fileExt)
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.DeleteAllFilesNeuroEEG(_sensorPtr, fileExt, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        public uint ReadFileCRC32NeuroEEG(string fileName, uint totalSize, uint offsetStart)
        {
            OpStatus opSt;
            uint val;
            byte error = SDKApiFactory.Inst.ReadFileCRC32NeuroEEG(_sensorPtr, fileName, totalSize, offsetStart, out val, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            return val;
        }
        public void FileStreamAutosaveNeuroEEG(string fileName)
        {
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.FileStreamAutosaveNeuroEEG(_sensorPtr, fileName, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }
        public void FileStreamReadNeuroEEG(string fileName, uint totalSize = 0xFFFFFFFF, uint offsetStart = 0)
        {
            if (totalSize == 0xFFFFFFFF)
            {
                var fInfo = ReadFileInfoNeuroEEG(fileName);
                if (offsetStart >= fInfo.FileSize)
                    return;
                totalSize = fInfo.FileSize - offsetStart;
            }
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.FileStreamReadNeuroEEG(_sensorPtr, fileName, totalSize, offsetStart, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
        }

        public static uint CalcCRC32(byte[] data)
        {
            return SDKApiFactory.Inst.CalcCRC32(data);
        }
        public static IntPtr CreateSignalProcessParamNeuroEEG(NeuroEEGAmplifierParam ampParam)
        {
            IntPtr val;
            OpStatus opSt;
            NeuroEEGAmplifierParamNative setter = new()
            {
                ReferentResistMesureAllow = ampParam.ReferentResistMesureAllow ? (byte)1 : (byte)0,
                Frequency = ampParam.Frequency,
                ReferentMode = ampParam.ReferentMode,
                ChannelMode = new EEGChannelMode[ampParam.ChannelMode.Length],
                ChannelGain = new SensorGain[ampParam.ChannelGain.Length],
                UseDiffAsRespiration = ampParam.UseDiffAsRespiration ? (byte)1 : (byte)0,
            };
            ampParam.ChannelMode.CopyTo(setter.ChannelMode, 0);
            ampParam.ChannelGain.CopyTo(setter.ChannelGain, 0);
            byte error = SDKApiFactory.Inst.CreateSignalProcessParamNeuroEEG(setter, out val, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            return val;
        }
        public static void RemoveSignalProcessParamNeuroEEG(IntPtr processParam)
        {
            SDKApiFactory.Inst.RemoveSignalProcessParamNeuroEEG(processParam);
        }
        public static uint ParseRawSignalNeuroEEG(byte[] data, IntPtr processParam, out SignalChannelsData[] signalOut, out ResistChannelsData[] resistOut)
        {
            uint szDataReadyOut;
            OpStatus opSt;
            byte error = SDKApiFactory.Inst.ParseRawSignalNeuroEEG(data, out szDataReadyOut, processParam, out signalOut, out resistOut, out opSt);
            SDKApiFactory.ThrowIfError(opSt, error);
            return szDataReadyOut;
        }

        public override void Dispose()
        {
            if (!_disposed)
            {
                SDKApiFactory.Inst.RemoveAmpModeCallback(_modeCallbackNeuroSmartHandle);
                SDKApiFactory.Inst.RemoveStimModeCallback(_modeCallbackNeuroStimHandle);
                SDKApiFactory.Inst.RemoveBatteryGaugeStateCallback(_batteryGaugeStateCallbackSensorHandle);
                SDKApiFactory.Inst.RemoveNeuroStimSyncStateCallback(_syncStateCallbackNeuroStimHandle);
                SDKApiFactory.Inst.RemoveSignalCallbackNeuroEEG(_signalCallbackNeuroEEGSensorHandle);
                SDKApiFactory.Inst.RemoveResistCallbackNeuroEEG(_resistCallbackNeuroEEGSensorHandle);
                SDKApiFactory.Inst.RemoveSignalResistCallbackNeuroEEG(_signalResistCallbackNeuroEEGSensorHandle);
                SDKApiFactory.Inst.RemoveSignalRawCallbackNeuroEEG(_signalRawCallbackNeuroEEGSensorHandle);
                SDKApiFactory.Inst.RemoveFileStreamReadCallbackNeuroEEG(_fileStreamReadCallbackNeuroEEGSensorHandle);
            }
            base.Dispose();
        }

#pragma warning disable CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.
        private static NeuroEEGSensor? SDKSensor(IntPtr userData)
        {
            if (userData != IntPtr.Zero)
            {
                var handle = GCHandle.FromIntPtr(userData);
                if (handle.IsAllocated)
                {
                    return handle.Target as NeuroEEGSensor;
                }
            }
            return null;
        }
#pragma warning restore CS8632 // The annotation for nullable reference types should only be used in code within a '#nullable' annotations context.

        [AOT.MonoPInvokeCallback(typeof(AmpModeCallbackSensor))]
        private static void AmpModeCallbackSensor(IntPtr ptr, SensorAmpMode mode, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventSensorAmpModeChanged?.Invoke(sensor, mode);
        }
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
        [AOT.MonoPInvokeCallback(typeof(SignalCallbackNeuroEEGSensor))]
        private static void SignalCallbackNeuroEEGSensor(IntPtr ptr, SignalChannelsDataNative[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);

            SignalChannelsData[] rxData = new SignalChannelsData[dataArray.Length];
            for (int i = 0; i < rxData.Length; ++i)
            {
                rxData[i].PackNum = dataArray[i].PackNum;
                rxData[i].Marker = dataArray[i].Marker;
                rxData[i].Samples = new double[dataArray[i].SzSamples];
                Marshal.Copy(dataArray[i].Samples, rxData[i].Samples, 0, rxData[i].Samples.Length);
            }
            sensor?.EventNeuroEEGSignalDataRecived?.Invoke(sensor, rxData);

        }
        [AOT.MonoPInvokeCallback(typeof(ResistCallbackNeuroEEGSensor))]
        private static void ResistCallbackNeuroEEGSensor(IntPtr ptr, ResistChannelsDataNative[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);

            ResistChannelsData[] rxData = new ResistChannelsData[dataArray.Length];
            for (int i = 0; i < rxData.Length; ++i)
            {
                rxData[i].PackNum = dataArray[i].PackNum;
                rxData[i].A1 = dataArray[i].A1;
                rxData[i].A2 = dataArray[i].A2;
                rxData[i].Ref = dataArray[i].Ref;
                rxData[i].Bias = dataArray[i].Bias;
                rxData[i].Values = new double[dataArray[i].SzValues];
                Marshal.Copy(dataArray[i].Values, rxData[i].Values, 0, rxData[i].Values.Length);
            }
            sensor?.EventNeuroEEGResistDataRecived?.Invoke(sensor, rxData);
        }
        [AOT.MonoPInvokeCallback(typeof(SignalResistCallbackNeuroEEGSensor))]
        private static void SignalResistCallbackNeuroEEGSensor(IntPtr ptr, SignalChannelsDataNative[] signalData, int szSignalData, ResistChannelsDataNative[] resistData, int szResistData, IntPtr userData)
        {
            var sensor = SDKSensor(userData);

            SignalChannelsData[] rxSignalData = new SignalChannelsData[signalData != null ? signalData.Length : 0];
            if (signalData != null)
            {
                for (int i = 0; i < rxSignalData.Length; ++i)
                {
                    rxSignalData[i].PackNum = signalData[i].PackNum;
                    rxSignalData[i].Marker = signalData[i].Marker;
                    rxSignalData[i].Samples = new double[signalData[i].SzSamples];
                    Marshal.Copy(signalData[i].Samples, rxSignalData[i].Samples, 0, rxSignalData[i].Samples.Length);
                }
            }

            ResistChannelsData[] rxResistData = new ResistChannelsData[resistData != null ? resistData.Length : 0];
            if (resistData != null)
            {
                for (int i = 0; i < rxResistData.Length; ++i)
                {
                    rxResistData[i].PackNum = resistData[i].PackNum;
                    rxResistData[i].A1 = resistData[i].A1;
                    rxResistData[i].A2 = resistData[i].A2;
                    rxResistData[i].Ref = resistData[i].Ref;
                    rxResistData[i].Bias = resistData[i].Bias;
                    rxResistData[i].Values = new double[resistData[i].SzValues];
                    Marshal.Copy(resistData[i].Values, rxResistData[i].Values, 0, rxResistData[i].Values.Length);
                }
            }
            sensor?.EventNeuroEEGSignalResistDataRecived?.Invoke(sensor, rxSignalData, rxResistData);
        }
        [AOT.MonoPInvokeCallback(typeof(SignalRawCallbackNeuroEEGSensor))]
        private static void SignalRawCallbackNeuroEEGSensor(IntPtr ptr, byte[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventNeuroEEGSignalRawDataRecived?.Invoke(sensor, dataArray);
        }
        [AOT.MonoPInvokeCallback(typeof(FileStreamReadCallbackNeuroEEGSensor))]
        private static void FileStreamReadCallbackNeuroEEGSensor(IntPtr ptr, SensorFileDataNative[] dataArray, int dataSize, IntPtr userData)
        {
            var sensor = SDKSensor(userData);

            SensorFileData[] rxData = new SensorFileData[dataArray.Length];
            for (int i = 0; i < rxData.Length; ++i)
            {
                rxData[i].OffsetStart = dataArray[i].OffsetStart;
                rxData[i].DataAmount = dataArray[i].DataAmount;
                rxData[i].Data = new byte[dataArray[i].SzData];
                Marshal.Copy(dataArray[i].Data, rxData[i].Data, 0, rxData[i].Data.Length);
            }
            sensor?.EventNeuroEEGFileStreamReadRecived?.Invoke(sensor, rxData);
        }
        [AOT.MonoPInvokeCallback(typeof(BatteryGaugeStateCallbackSensor))]
        private static void BatteryGaugeStateCallbackSensor(IntPtr ptr, SensorBatteryGauge battGauge, IntPtr userData)
        {
            var sensor = SDKSensor(userData);
            sensor?.EventBatteryGaugeStateChanged?.Invoke(sensor, battGauge);
        }
    }
}