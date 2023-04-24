package com.neurosdk2.ble;

import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;


public class BLEBroadcastReceiver extends BroadcastReceiver {
    private long nativePointer;
    private HandlerThread bgThread;

    public void Register(Context context){
        if(bgThread.isAlive()) return;

        bgThread.start();
        Looper looper = bgThread.getLooper();
        Handler broadcastHandler = new Handler(looper);

        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        context.registerReceiver(this,filter,null,broadcastHandler);
    }
    public void Unregister(Context context){
        try{
            context.unregisterReceiver(this);
        }catch (Exception ex)
        {
            int i = 0;
        }
    }
    public BLEBroadcastReceiver(long nativePointer) {
        this.nativePointer = nativePointer;
        this.bgThread = new HandlerThread(String.valueOf(nativePointer));
    }

    @Override
    public void onReceive(Context context, Intent intent)
    {
        if (intent != null)
        {
            Bundle extras = intent.getExtras();
            if (extras != null)
            {
                int bondState = extras.getInt(BluetoothDevice.EXTRA_BOND_STATE);
                BluetoothDevice device = extras.getParcelable(BluetoothDevice.EXTRA_DEVICE);
                if(bondState != 0 && device != null){
                    OnBondStateChanged(nativePointer,bondState,device.getAddress());
                }
            }
        }
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        bgThread.quit();
        OnBondReceiverDestroyed(nativePointer);
    }

    native void OnBondStateChanged(long pointer, int bondState, String deviceAddress);
    native void OnBondReceiverDestroyed(long pointer);
}
