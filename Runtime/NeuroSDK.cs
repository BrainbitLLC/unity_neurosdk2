using System;
using System.Collections.Generic;

namespace NeuroSDK
{
    public interface IScanner : IDisposable
    {
        /// <summary>
        /// List of found devices
        /// </summary>
        IReadOnlyList<SensorInfo> Sensors { get; }
        /// <summary>
        /// Sensor List Change Event
        /// </summary>
        event SensorsChanged EventSensorsChanged;
        /// <summary>
        /// Start search sensors
        /// </summary>
        void Start();
        /// <summary>
        /// Stop search sensors
        /// </summary>
        void Stop();
        /// <summary>
        /// Creating a sensor
        /// </summary>
        /// <param name="sensorInfo">Sensor Information</param>
        /// <returns>Created sensor</returns>
        ISensor CreateSensor(SensorInfo sensorInfo);
    }

    public interface ISensor : IDisposable
    {
        /// <summary>
        /// Type of device
        /// </summary>
        SensorFamily SensFamily { get; }
        /// <summary>
        /// Specific set of modules
        /// </summary>
        IReadOnlyList<SensorFeature> Features { get; }
        /// <summary>
        /// List of supported commands
        /// </summary>
        IReadOnlyList<SensorCommand> Commands { get; }
        /// <summary>
        /// Sensor Parameters
        /// </summary>
        IReadOnlyList<ParameterInfo> Parameters { get; }
        /// <summary>
        /// Sensor Name
        /// </summary>
        string Name { get; set; }
        /// <summary>
        /// Sensor State
        /// </summary>
        SensorState State { get; }
        /// <summary>
        /// Sensor Address
        /// </summary>
        string Address { get; }
        /// <summary>
        /// Sensor SerialNumber
        /// </summary>
        string SerialNumber { get; set; }
        /// <summary>
        /// Sensor charge level
        /// </summary>
        int BattPower { get; }
        /// <summary>
        /// Sensor Version
        /// </summary>
        SensorVersion Version { get; }

        SensorSamplingFrequency SamplingFrequency { get; }
        SensorGain Gain { get; }
        SensorDataOffset DataOffset { get; }
        SensorFirmwareMode FirmwareMode { get; }

        ///// <summary>
        ///// Headband Sensor IrAmplitude
        /// <summary>
        /// Battery level change event.
        /// </summary>
        event BatteryChanged EventBatteryChanged;
        /// <summary>
        /// Connection status change event.
        /// </summary>
        event SensorStateChanged EventSensorStateChanged;
        /// <summary>
        /// Connect to Sensor
        /// </summary>
        void Connect();
        /// <summary>
        /// Disconnect to Sensor
        /// </summary>
        void Disconnect();
        /// <summary>
        /// Checking the sensor feature support
        /// </summary>
        /// <param name="sensorFeature">A verifiable feature.</param>
        bool IsSupportedFeature(SensorFeature sensorFeature);
        /// <summary>
        /// Checking the sensor command support
        /// </summary>
        /// <param name="sensorCommand">A verifiable command.</param>
        bool IsSupportedCommand(SensorCommand sensorCommand);
        /// <summary>
        /// Checking the sensor parameter support
        /// </summary>
        /// <param name="sensorParameter">A verifiable parameter.</param>
        bool IsSupportedParameter(SensorParameter sensorParameter);
        /// <summary>
        /// Execute a command on the sensor
        /// </summary>
        /// <param name="sensorCommand">Command to execute.</param>
        void ExecCommand(SensorCommand sensorCommand);
    }

    public class SDKException : InvalidOperationException
    {
        public uint ErrorCode
        {
            get;
            private set;
        }
        public SDKException(string message, uint code) : base(message)
        {
            ErrorCode = code;
        }
        public SDKException(string message) : base(message)
        {
        }
    }
}
