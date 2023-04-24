package com.neurosdk2.ble;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;

public class BleGattCallback extends BluetoothGattCallback
{
    private final long mNativeCallbackPtr;

    public BleGattCallback(long nativeCallbackPtr)
    {
        mNativeCallbackPtr = nativeCallbackPtr;
    }

    @Override
    public void finalize()
    {
        destroy(mNativeCallbackPtr);
    }

    @Override
    public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic)
    {
        String characteristicUUID = characteristic.getUuid().toString();
        String seriveUUID = characteristic.getService().getUuid().toString();
        byte[] data = characteristic.getValue();
        onCharacteristicChanged(mNativeCallbackPtr, characteristicUUID,seriveUUID,data);
    }

    @Override
    public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState){
        onConnectionStateChange(mNativeCallbackPtr, status, newState);
    }

    @Override
    public void onServicesDiscovered(BluetoothGatt gatt, int status){
        onServicesDiscovered(mNativeCallbackPtr, status);
    }

    @Override
    public void onCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status){

        onCharacteristicRead(mNativeCallbackPtr, characteristic.getUuid().toString(), status);
    }

    @Override
    public void onCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status){
        onCharacteristicWrite(mNativeCallbackPtr, characteristic.getUuid().toString(), status);
    }

    @Override
    public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status){

        onDescriptorWrite(mNativeCallbackPtr, descriptor.getUuid().toString(),descriptor.getCharacteristic().getUuid().toString(), status);
    }

    private static native void destroy(long nativeCallbackPtr);
    private static native void onCharacteristicChanged(long nativeCallbackPtr, String uuid,String serviceUuid, byte[] data);
    private static native void onCharacteristicRead(long nativeCallbackPtr, String uuid, int status);
    private static native void onCharacteristicWrite(long nativeCallbackPtr, String uuid, int status);
    private static native void onDescriptorWrite(long nativeCallbackPtr, String uuid, String charUUID,int status);
    private static native void onConnectionStateChange(long nativeCallbackPtr, int status, int newState);
    private static native void onServicesDiscovered(long nativeCallbackPtr, int status);
}
