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

### Signal Callibri, Kolibri

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
