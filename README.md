# Overview

Neurosdk is a powerful tool for working with neuro-sensors BrainBit, BrainBitBlack, Callibri and Kolibri. All these devices work with BLE 4.2+ technology. Unity SDK is designed for Android, iOS, Windows and MacOS platforms. SDK allows you to connect, read the parameters of devices, as well as receive signals of various types from the selected device. 

> The library is designed for Unity 2020 and higher. You can use it on versions below at your own risk.

# Getting Started

## Android:

> The Android version is designed for APIs >= 21.

Because sdk uses the bluetooth api you need to set up the project. 

For Android you need to add permissions to use geolocation and bluetooth to the manifest. You can read more about this on [AndroidDeveloper](https://developer.android.com/guide/topics/connectivity/bluetooth/permissions).

Firstly, enable custom `AndroidManifest`:
1. Go to `PlayerSettings`
2. Open `Android` tab, then find `PublishingSettings`, then find `Build` part
3. Enable `Custom Main Manifest` option
4. Add permissions into it:

```
<uses-permission android:name="android.permission.ACCESS_BACKGROUND_LOCATION" />
<uses-permission android:name="android.permission.ACCESS_COARSE_LOCATION" />
<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
<uses-permission android:name="android.permission.BLUETOOTH" android:maxSdkVersion="30" />
<uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
<uses-permission android:name="android.permission.BLUETOOTH_ADVERTISE" />
<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />
<uses-permission android:name="android.permission.BLUETOOTH_SCAN" tools:targetApi="31" android:usesPermissionFlags="neverForLocation" />
```

For android platform ask for bluetooth and location permission before use:

```csharp
#if UNITY_ANDROID
            if (SystemInfo.operatingSystem.Contains("31") ||
                SystemInfo.operatingSystem.Contains("32") ||
                SystemInfo.operatingSystem.Contains("33"))
            {
                Permission.RequestUserPermission("android.permission.BLUETOOTH_SCAN");
                Permission.RequestUserPermission("android.permission.BLUETOOTH_CONNECT");
            }
            else
            {
                Permission.RequestUserPermission("android.permission.ACCESS_FINE_LOCATION");
                Permission.RequestUserPermission("android.permission.ACCESS_COARSE_LOCATION");
            }
#endif
```

## iOS

> The iOS version is designed for APIs >= 12.0.

For iOS >= 13.0 you need to add a key to `Info.plist`:

```
<key>NSBluetoothAlwaysUsageDescription</key>
<string>Explanation to using bluetooth</string>
```

And for iOS < 13.0:

```
<key>NSBluetoothPeripheralUsageDescription</key>
<string>Explanation to using bluetooth</string>
```

You can do it manually after built project by adding values to `Info.plist` or by the `OnPostProcessBuild` script like following:

```csharp
var infoPlist = new PlistDocument();
var infoPlistPath = path + "/Info.plist";
infoPlist.ReadFromFile(infoPlistPath);

PlistElementDict dict = infoPlist.root.AsDict();
dict.SetString("NSBluetoothAlwaysUsageDescription", "App requires access to Bluetooth to allow you connect to device");
dict.SetString("NSBluetoothPeripheralUsageDescription", "App uses Bluetooth to connect with your Brainbit device");
infoPlist.WriteToFile(infoPlistPath);
```

> After build you need to change Embed type to `neurosdk2.framework` from `Embed Without Signing` to `Embed & Sign` into `Unity-iPhone` module.

# Usage
 
## Scanner

The scanner works like this:

1. Create scanner. 
When creating a scanner, you need to specify the type of device to search. It can be either one device or several. Here is example for two type of devices - BrainBit and Callibri.

```csharp
Scanner scanner = new Scanner(SensorFamily.SensorLECallibri, SensorFamily.SensorLEBrainBit);
```

2. During the search, you can get a list of found devices using a callback. To do this, you need to subscribe to receive the event, and unsubscribe after the search is completed:

```csharp
private void Scanner_Founded(IScanner scanner, IReadOnlyList<SensorInfo> sensors)
{
    foreach(SensorInfo sensorInfo in sensors){
        Console.WriteLine(sensorInfo.Name + ": " + sensorInfo.Address);
    }
}

scanner.EventSensorsChanged += Scanner_Founded;
...
scanner.EventSensorsChanged -= Scanner_Founded;
```

3. Start and stop search

```csharp
scanner.Start();
...
scanner.Stop();
```

4. Additionally, a list of found devices can be obtained using a separate method.

```csharp
IReadOnlyList<SensorInfo> sensors = scanner.Sensors;
```

`SensorInfo` contains information about device:

| Field | Type | Description |
|--|--|--|
|Name|string|the name of device|
|Address|string|MAC address of device (UUID for iOS/MacOS)|
|SerialNumber|string|device's serial number|
|SensFamily|SensorFamily|type of device|
|SensModel|byte|numerical value of the device model|
|PairingRequired|bool|whether the device needs to be paired or not|
|RSSI|short|current signal strength in dBm. The valid range is [-127, 126]|

5. After you finish working with the scanner, you need to clean up the resources used. 

```csharp
scanner.Dispose();
```

> Important!
> When restarting the search, the callback will only be called when a new device is found. If you need to get all devices found by the current scanner instance, call the appropriate method.

## Sensor

You need to create a device using a scanner. All manipulations with the device will be performed without errors only if the device is connected.

```csharp
Sensor sensor = scanner.CreateSensor(sensorInfo);
```

> Device creation is a blocking method, so it must be called from separate thread.

### Manage connectionState

Connection status can be obtained in two ways. The first one is using the sensor property `State`.

The second way is in real time using a callback:

```csharp
 private void EventSensorStateChanged(ISensor sensor, SensorState sensorState)
{
    Console.WriteLine(sensorState);
}
...
sensor.EventSensorStateChanged += EventSensorStateChanged;
...
sensor.EventSensorStateChanged -= EventSensorStateChanged;
```

A connection can be in two states: connected (InRange) and disconnected (OutOfRange).

> Important!
> The state change callback will not come after the device is created, only after disconnecting (device lost from the scope or using a method) and then connected. The device does not automatically reconnect.

You can connect and disconnect from device manually by methods `Connect()` and `Disconnect()`. To receive connection state in real time you need to subscribe to `stateChanged` event. Also you can get connection state by sensor's property.

```csharp
sensor.Disconnect();
...
sensor.Connect();
```

### Battery

Also, you can get power value from each device by sensor property `BattPower` or by callback in real time:

```csharp
private void EventBatteryChanged(ISensor sensor, int battPower)
{
    Console.WriteLine("Power: " + battPower);
}
...
sensor.EventBatteryChanged += EventBatteryChanged;
...
sensor.EventBatteryChanged -= EventBatteryChanged;
```

### Parameters

Each device has its own settings, and some of them can be configured as you need. Not all settings can be changed and not every device supports all settings.

Firstly you need to find out what parameters the device supports and whether they can be changed:

```csharp
var parameters = sensor.Parameters;
```

Info about parameter includes two fields:
 - the name of the parameter, represented by an enumeration
 - parameter availability for manipulation. Can be one of three values:
   - read - read-only
   - read and write - parameter can be changed
   - read and notify - parameter is updated over time

You can also check if the parameter is supported, for example `Gain`:

```csharp
if(sensor.IsSupportedParameter(SensorParameter.ParameterGain)){
...
}
```

Each device has a specific set of modules. You can find out which modules the device has using the property `Feature`:

```csharp
var features = sensor.Features;
```

You can also check if the feature is supported, for example `Signal`:

```csharp
if(sensor.IsSupportedFeature(SensorFeature.FeatureSignal)){
...
}
```

The device can execute certain commands. The list of supported commands can be obtained as follows:

```csharp
var commands = sensor.Commands;
```

And also check if the device can execute the desired command:

```csharp
if(sensor.IsSupportedCommand(SensorCommand.StartSignal)){
...
}
```

### BrainBit, BrainBitBlack

The BrainBit and BrainBitBlack is a headband with 4 electrodes and 4 data channels - O1, O2, T3, T4. The device has a frequency of 250 Hz, which means that data on each of the channels will come at a frequency of 250 samples per second. The parameters of this device, such as gain, data offset and the other, cannot be changed, if you try to do this, an exception will appear.

```csharp
sensor.Gain = SensorGain.SensorGain1; // <- This throw an exeption!
```

>  You can distinguish BrainBit device from Flex by the firmware version number: if the `SensorVersion.FwMajor` is more than 100 - it's Flex, if it's less than BrainBit.

> BrainBitBlack, unlike BrainBit, requires pairing with a PC/mobile device. So, before connecting to the BBB, you must put it into pairing mode. SDK starts the pairing process automatically.

To use the BrainBit features, you need to bring the sensor type to a BrainBit or BrainBitBlack device type. The sensor as such does not provide signal or resistance values.

```charp
BrainBitSensor sensor = scanner.CreateSensor(info) as BrainBitSensor;
// or
BrainBitBlackSensor sensor = scanner.CreateSensor(info) as BrainBitBlackSensor;
// or
var sensor = = scanner.CreateSensor(info);
// or
Sensor sensor = scanner.CreateSensor(info);
BrainBitSensor BBsensor = sensor as BrainBitSensor;
```

#### Receiving signal

To receive signal data, you need to subscribe to the corresponding callback. The values will be received as a packet from four channels at once, which will avoid desynchronization between them. The values come in volts. In order for the device to start transmitting data, you need to start a signal using the `execute` command. This method is also recommended to be run in an separate thread.

```csharp
private void onBrainBitSignalDataRecived(ISensor sensor, BrainBitSignalData[] data)
{
    Console.WriteLine("Data: " + data);
}
...
sensor.EventBrainBitSignalDataRecived += onBrainBitSignalDataRecived;
sensor.ExecCommand(SensorCommand.CommandStartSignal);
...
sensor.EventBrainBitSignalDataRecived -= onBrainBitSignalDataRecived;
sensor.ExecCommand(SensorCommand.CommandStopSignal);
```

#### Recieving resistance

BrainBit and BrainBitBlack also allow you to get resistance values. With their help, you can determine the quality of the electrodes to the skin. 

```csharp
private void onBrainBitResistDataRecived(ISensor sensor, BrainBitResistData data)
{
    Console.WriteLine("Data: " + data);
}
...
sensor.EventBrainBitResistDataRecived += onBrainBitResistDataRecived;
sensor.ExecCommand(SensorCommand.CommandStartResist);
...
sensor.EventBrainBitResistDataRecived -= onBrainBitResistDataRecived;
sensor.ExecCommand(SensorCommand.CommandStopResist);
```

### BrainBit 2/Flex/Pro

The BrainBit2 class is designed to work with several device families: BrainBit 2, BrainBit Pro, BrainBit Flex, BrainBit Flex Pro. All devices have a sampling frequency of 250Hz. All devices can work in two modes - signal and resistance separately. These devices have different number of channels - BrainBit2, BrainBit Flex have 4 channels each and BrainBitPro, BrainBit FlexPro have 8 channels each. The main difference from BraibBit of the first version is that they do not support gain property, but have the ability to set gain for each channel separately using `BrainBit2AmplifierParam` structure. Also, these types of devices support the [ping command](#ping-signal).

#### Info about channels

The device can have 4 or 8 channels. The number of channels can be determined as follows:

```csharp
IReadOnlyList<EEGChannelInfo> channels = sensor.SupportedChannelsBrainBit2;
```

`EEGChannelInfo` contains some info:

| Field | Type | Description |
|--|--|--|
|Id|EEGChannelId|physical location of the channel. You will receive the values `O1`, `O2`, `T3`, `T4` or `Unknown`. `Unknown` means that the position of a specific electrode is free.|
|ChType|EEGChannelType|type of channel, possible values `A1`, `A2`, `Differential` or `Referent`|
|Name|string|channel name|
|Num|byte|channel number. By this number the channel will be located in the array of signal or resistance values|

Also you can check only channels count without info:

```csharp
int chCount = sensor.ChannelsCount;
```

#### AmpMode

This device can show it's current amplifier mode. It can be in the following states:

  - Invalid
  - PowerDown
  - Idle
  - Signal
  - Resist

You can check amp. mode by two ways:

1. by callback:

```csharp
private void SensorEventSensorAmpModeChanged(ISensor sensor, SensorAmpMode sensorAmpMode)
{
    Console.Writeline($"AmpMode: {sensorAmpMode}");
}
...
sensor.EventSensorAmpModeChanged += SensorEventSensorAmpModeChanged;
...
sensor.EventSensorAmpModeChanged -= SensorEventSensorAmpModeChanged;
```

2. get value at any time:

```csharp
SensorAmpMode ampMode = sensor.AmpMode;
```

It is very important parameter for BrainBit2 device because you can set amplifier parameters only if device into `PowerDown` or `Idle` mode.

#### Amplifier parameters

You can configure each channel and whole device settings by setting amplifier parameters.

```csharp
BrainBit2AmplifierParam ampParam = sensor.AmplifierParamBrainBit2;
for (int i = 0; i < ampParam.ChSignalMode.Length; ++i)
{
    ampParam.ChSignalMode[i] = BrainBit2ChannelMode.ChModeNormal;
    ampParam.ChGain[i] = SensorGain.SensorGain6;
    ampParam.ChResistUse[i] = true;
}
ampParam.Current = GenCurrent.GenCurr6nA;
sensor.AmplifierParamBrainBit2 = ampParam;
```

`BrainBit2AmplifierParam` contains:

| Field | Type | Description |
|--|--|--|
|ChSignalMode|BrainBit2ChannelMode[]|input type|
|ChResistUse|bool[]|is use for resistance|
|ChGain|SensorGain[]|gain of an ADC signal for each channel|
|Current|GenCurrent|setting parameters of the probe current generator|

Possible values for `Current`:
 - 0nA 
 - 6nA 
 - 12nA
 - 18nA
 - 24nA

Signal modes:
 - Short - shorted input
 - Normal - bipolar input mode (used for EEG)

Possible `Gain` values:
 - 1
 - 2
 - 3
 - 4
 - 6
 - 8
 - 12

#### Receiving signal

To receive signal data, you need to subscribe to the corresponding callback. The values come in volts. In order for the device to start transmitting data, you need to start a signal using the `execute` command. This method is also recommended to be run in an separate thread.

```csharp
private static void SensorEventBrainBit2SignalDataRecived(ISensor sensor, SignalChannelsData[] data)
{
    foreach (var it in data)
    {
        Console.WriteLine($"[{it.PackNum}][{it.Marker}][{it.Samples[0]}][{it.Samples[1]}][{it.Samples[2]}][{it.Samples[3]}]");
    }
}
...
sensor.EventBrainBit2SignalDataRecived += SensorEventBrainBit2SignalDataRecived;
sensor.ExecCommand(SensorCommand.CommandStartSignal);
...
sensor.EventBrainBit2SignalDataRecived -= SensorEventBrainBit2SignalDataRecived;
sensor.ExecCommand(SensorCommand.CommandStopSignal);
```

You get signal values as a list of samples, each containing:

| Field | Type | Description |
|--|--|--|
|PackNum|uint|number for each packet|
|Marker|byte|marker of sample|
|Samples|double[]|array of samples in V. Each sample number into array consistent with `Num` value of [EEGChannelInfo](#info-about-channels) from `getSupportedChannels()` method.|

#### Receiving resistance

BrainBit2 also allow you to get resistance values. With their help, you can determine the quality of the electrodes to the skin. Initial resistance values are infinity. The values change when the device is on the head.

```csharp
private static void SensorEventBrainBit2ResistDataRecived(ISensor sensor, ResistRefChannelsData[] data)
{
    if (data.Length < 1) return;
    ResistRefChannelsData lastData = data[data.Length - 1];
    Console.WriteLine($"0: {lastData.Samples[0]} 1: {lastData.Samples[1]} 2: {lastData.Samples[2]} 3: {lastData.Samples[3]}");
}
...
sensor.EventBrainBit2ResistDataRecived += SensorEventBrainBit2ResistDataRecived;
sensor.ExecCommand(SensorCommand.CommandStartResist);
...
sensor.EventBrainBit2ResistDataRecived -= SensorEventBrainBit2ResistDataRecived;
sensor.ExecCommand(SensorCommand.CommandStopResist);
```

You get resistance values structure of samples (`ResistRefChannelsData`) for each channel:

| Field | Type | Description |
|--|--|--|
|PackNum|uint|number for each packet|
|Samples|double[]|array of samples in V. Each sample number into array consistent with `Num` value of [EEGChannelInfo](#info-about-channels) from `readSupportedChannelsBrainBit2()` method.|
|Referents|double[]|array of values for referents channels. For BrainBit2 sensor is empty now.|

### Callibri, Kolibri MF

The Callibri family of devices has a wide range of built-in modules. For each of these modules, the SDK contains its own processing area. It is recommended before using any of the modules to check if the module is supported by the device using one of the methods `IsSupportedFeature`, `IsSupportedCommand` or `IsSupportedParameter`

To use the Callibri features, you need to bring the sensor type to a Callibri device type. The sensor as such does not provide signal or electrodes state values.

```charp
CallibriSensor sensor = scanner.CreateSensor(info) as CallibriSensor;
// or
var sensor = = scanner.CreateSensor(info);
// or
Sensor sensor = scanner.CreateSensor(info);
CallibriSensor Сsensor = sensor as CallibriSensor;
```

#### Receiving signal

To receive signal data, you need to subscribe to the corresponding callback. The values come in volts. In order for the device to start transmitting data, you need to start a signal using the `execute` command. This method is also recommended to be run in an separate thread.

The sampling rate can be controlled using the `SamplingFrequency` property. For example, at a frequency of 1000 Hz, the device will send 1000 samples per second. Supports frequencies 125/250/500/1000/2000/4000/8000 Hz. You can also adjust the signal offset (`DataOffset`) and signal power (`Gain`).

```csharp
private void onCallibriSignalDataRecived(ISensor sensor, CallibriSignalData[] data)
{
    Console.WriteLine("Data: " + data);
}
...
sensor.EventCallibriSignalDataRecived += onCallibriSignalDataRecived;
sensor.ExecCommand(SensorCommand.CommandStartSignal);
...
sensor.EventCallibriSignalDataRecived -= onCallibriSignalDataRecived;
sensor.ExecCommand(SensorCommand.CommandStopSignal);
```

Callibri can output envelope and respiration data. These channels have a frequency of 20 Hz. Some models support MEMS, its frequency is 100 Hz.

#### Check electrodes state

Checking the condition of the electrodes is used to determine the quality of the electrodes to the skin. It can be in three states - adjacent (`Normal`) / not adjacent (`Detached`) / resistance is too high (`HighResistance`). To receive data, you need to subscribe to the corresponding callback and start signal pickup.

```csharp
private void onCallibriElectrodeStateChanged(ISensor sensor, CallibriElectrodeState elState)
{
    Console.WriteLine(elState);
}
...
sensor.EventCallibriElectrodeStateChanged += onCallibriElectrodeStateChanged;
sensor.ExecCommand(SensorCommand.CommandStartSignal);
...
sensor.EventCallibriElectrodeStateChanged -= onCallibriElectrodeStateChanged;
sensor.ExecCommand(SensorCommand.CommandStopSignal);
```
#### MEMS

The MEMS microcircuit is optional on request. Its presence can be checked using the [IsSupportedFeature](#features) method. This means that the device contains an accelerometer and a gyroscope. Contains information about the position of the device in space. Channel sampling frequency is 100 Hz.

MEMS data is a structure:

- PackNum - number for each packet
- Accelerometer - accelerometer data. Contains:
  - X - Abscissa Acceleration
  - Y - Y-axis acceleration
  - Z - Acceleration along the applicate axis
- Gyroscope - gyroscope data
  - X - The angle of inclination along the abscissa axis
  - Y - Inclination angle along the ordinate axis
  - Z - Angle of inclination along the axis of the applicate

Quaternion data is a structure:

- PackNum - number for each packet
- W - Rotation component
- X - Vector abscissa coordinate
- Y - Vector coordinate along the ordinate axis
- Z - The coordinate of the vector along the axis of the applicate

It is recommended to perform calibration on a flat, horizontal non-vibrating surface before starting work using the `CalibrateMEMS` command. Calibration state can be checked using the [MEMSCalibrateStateCallibri](#mems-calibration-status) property, it can take only two values: calibrated (true), not calibrated (false).

> MEMS and quaternion available only to signal Callibri/Kolibri!

```csharp
private void Sensor_EventQuaternionDataRecived(ISensor sensor, QuaternionData[] data)
{
    Console.WriteLine(data.ToString());
}

private void Sensor_EventMEMSDataRecived(ISensor sensor, MEMSData[] data)
{
    Console.WriteLine(data.ToString());
}
...
// Calibration
sensor.ExecCommand(SensorCommand.CommandCalibrateMEMS);
bool isCalibrated = sensor.MEMSCalibrateStateCallibri;

// For receiving MEMS
sensor.EventMEMSDataRecived += Sensor_EventMEMSDataRecived;
sensor.ExecCommand(SensorCommand.CommandStartMEMS);
...
sensor.EventMEMSDataRecived -= Sensor_EventMEMSDataRecived;
sensor.ExecCommand(SensorCommand.CommandStopMEMS);

// For quaternion
sensor.EventQuaternionDataRecived += Sensor_EventQuaternionDataRecived; 
sensor.ExecCommand(SensorCommand.CommandStartAngle);
...
sensor.EventQuaternionDataRecived -= Sensor_EventQuaternionDataRecived; 
sensor.ExecCommand(SensorCommand.CommandStopAngle);
```

#### Motion counter

Represents a motion counter. It can be configured using the [CallibriMotionCounterParam](#motion-counter) property, in it:

 - InsensThreshmG – Threshold of the algorithm's deadness in amplitude (in mg). The maximum value is 500mg. The minimum value is 0.
 - InsensThreshSamp - Threshold of the algorithm's insensitivity in time (in samples with the MEMS sampling rate). The maximum value is 500 samples. The minimum value is 0.

You can find out the current number of movements using the `MotionCounterCallibri` property. You can reset the counter with the `ResetMotionCounter` command. No additional commands are needed to start the counter, it will be incremented all the time until the reset command is executed.

```csharp
if(sensor.IsSupportedParameter(SensorParameter.ParameterMotionCounter))
{
    sensor.MotionCounterParamCallibri = new CallibriMotionCounterParam() { 
                    InsenseThresholdMG = 250,
                    InsenseThresholdSample = 250
                    };
    int motionCount = sensor.MotionCounterCallibri;
    sensor.ExecCommand(SensorCommand.CommandResetMotionCounter);
}
```

### Callibri/Kolibri EMS

Kallibri is a EMS if it supports the stimulation module:

```csharp
bool isStimulator = sensor.IsSupportedFeature(SensorFeature.FeatureCurrentStimulator);
```

#### Stimulation

Before starting the session, you need to correctly configure the device, otherwise the current strength may be too strong or the duration of stimulation too long. The setting is done using the `StimulatorParamCallibri` property. You can set the following options:

 - Current - stimulus amplitude in  mA. 1..100
 - PulseWidth - duration of the stimulating pulse by us. 20..460
 - Frequency - frequency of stimulation impulses by Hz. 1..200.
 - StimulusDuration - maximum stimulation time by ms. 0...65535. Zero is infinitely.

You can start and stop stimulation with the following commands:

```csharp
sensor.ExecCommand(SensorCommand.CommandStartCurrentStimulation);
...
sensor.ExecCommand(SensorCommand.CommandStopCurrentStimulation);
```

> Stimulation does not stop after the `StimulusDuration` time has elapsed.

You can check the state of stimulation using the [StimulatorMAStateCallibri](#stimulator-and-motional-assistant-state) property. Contains two parameters:

- StimulatorState - Stimulation mode state
- MAState - Drive assistant mode state

Each of the parameters can be in 4 states:

- NoParams - parameter not set
- Disabled - mode disabled
- Enabled - mode enabled
- Unsupported - sensor unsupported

#### Motion assistant

The Callibri EMS, which contains the MEMS module, can act as a motion corrector. You can set the initial and final angle of the device and the limb on which the Callibri/Kolibri is installed, as well as a pause between stimulations and turn on the motion assistant. All the time while the device is tilted in the set range, stimulation will met. Stimulation will take place according to the settings of [StimulatorParamCallibri](#stimulator-parameter-pack).

The motion corrector works in the background. After turning on the motion assistant mode, it will work regardless of the connection to a mobile device/PC. You can turn on the motion corrector mode using a special command. When the device is rebooted, it is also reset.

Motion corrector parameters are a structure with fields:

 - GyroStart - Angle value in degrees at which the stimulation mode will start, if it is correctly configured.
 - GyroStop - Angle value in degrees above which the stimulation mode will stop, if it is correctly configured.
 - Limb - overlay location in stimulation mode, if supported.
 - MinPauseMs - Pause between starts of stimulation mode in milliseconds. Multiple of 10. This means that the device is using the (MinPauseMs / 10) value. Correct values: 10, 20, 30, 40 ... 

```csharp
sensor.MotionAssistantParamCallibri = new CallibriMotionAssistantParams() { 
                    GyroStart = (byte)45,
                    GyroStop = 10,
                    Limb = CallibriMotionAssistantLimb.MALimbRightLeg,
                    MinPauseMs = 10 };
sensor.ExecCommand(SensorCommand.CommandEnableMotionAssistant);
...
sensor.ExecCommand(SensorCommand.CommandDisableMotionAssistant);
```

##### Parameters

###### Stimulator and motional assistant state

Parameter for obtaining information about the state of the stimulation mode and the motion assistant mode. This parameter is available only to Callibi/Kolibri EMS. Contains:

- StimulatorState - Stimulation mode state
- MAState - Drive assistant mode state

Each of the fields can be in four states:

| Field | Type | Description |
|--|--|--|
|StimStateEnabled|2|enabled|
|StimStateDisabled|1|disabled|
|StimStateNoParams|0|not set|
|StimStateUnsupported|0xFF|unsupported|

```csharp
// check supported future
if(sensor.IsSupportedFeature(SensorFeature.FeatureCurrentStimulator)){
    // param supported
}

// get
CallibriStimulatorMAState stimMAState = sensor.StimulatorMAStateCallibri;
```

###### Stimulator parameter pack

Stimulation parameters. This property is available only to Callibi/Kolibri EMS. Parameters inncludes fields:

**CallibriStimulationParams:**

| Field | Type | Description |
|--|--|--|
|Current|byte|stimulus amplitude in  mA. 1..100|
|PulseWidth|ushort|duration of the stimulating pulse by us. 20..460|
|Frequency|byte|frequency of stimulation impulses by Hz. 1..200|
|StimulusDuration|ushort|maximum stimulation time by ms. 0...65535|

```csharp
CallibriStimulationParams params = sensor.StimulatorParamCallibri
...
// set
CallibriStimulationParams params = new CallibriStimulationParams() { 
                    Current = (byte)100,
                    PulseWidth = (ushort)30,
                    Frequency = (byte)300,
                    StimulusDuration = (ushort)1
                    };
sensor.StimulatorParamCallibri = params
```


###### Motion assistant parameter pack

Parameter for describing a stimulation mode, if the device supports this mode. This structure describes the parameters for starting the stimulation mode depending on the place of application of the device, the position of the limb in space and the pause between the starts of this mode while the specified conditions are met. The parameter is available only to Callibi/Kolibri EMS. Parameters include fields:

**CallibriMotionAssistantParams:**
| Field | Type | Description |
|--|--|--|
|GyroStart|byte|Angle value in degrees at which the stimulation mode will start, if it is correctly configured|
|GyroStop|byte|Angle value in degrees above which the stimulation mode will stop, if it is correctly configured|
|Limb|CallibriMotionAssistantLimb|overlay location in stimulation mode, if supported|
|MinPauseMs|byte|Pause between starts of stimulation mode in milliseconds. Multiple of 10. This means that the device is using the (MinPauseMs / 10) value. Correct values: 10, 20, 30, 40 |

**CallibriMotionAssistantLimb:**
| Field | Value| Description |
|--|--|--|
|MALimbRightLeg|0|right leg|
|MALimbLeftLeg|1|left leg|
|MALimbRightArm|2|right arm|
|MALimbLeftArm|3|left arm|

```csharp
// get
CallibriMotionAssistantParams params = sensor.MotionAssistantParamCallibri;

// set
CallibriMotionAssistantParams params = new CallibriMotionAssistantParams(){
    GyroStart = (byte)30, 
    GyroStop = (byte)90, 
    Limb = CallibriMotionAssistantLimb.RightLeg, 
    MinPauseMs = (byte)10
};
sensor.MotionAssistantParamCallibri = params
```
