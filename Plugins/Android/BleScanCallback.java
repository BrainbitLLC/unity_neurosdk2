package com.neurosdk2.ble;

import android.annotation.SuppressLint;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;

import java.util.List;

public class BleScanCallback extends ScanCallback {
    private final String TAG = "BleScanCallback";
    private long nativePointer = 0;

    public BleScanCallback(long pointer) {
        this.nativePointer = pointer;
    }


    @SuppressLint("MissingPermission")
    @Override
    public void onScanResult(int callbackType, ScanResult result) {
        super.onScanResult(callbackType, result);
        try {
            if(nativePointer == 0) return;
            OnScanResult(nativePointer,callbackType,result);
        } catch (Exception e) {
        }
    }

    @Override
    public void onBatchScanResults(List<ScanResult> results) {
        super.onBatchScanResults(results);
        try {
            if(nativePointer == 0) return;
            if(!results.isEmpty())
            {
                OnBatchScanResult(nativePointer,results);
            }
        } catch (Exception e) {
        }
    }

    @Override
    public void onScanFailed(int errorCode) {
        super.onScanFailed(errorCode);
        try {
            if(nativePointer == 0) return;
            OnScanFailed(nativePointer,errorCode);
        } catch (Exception e) {
        }
    }
    @Override
    protected void finalize() throws Throwable {
        OnDestroy(nativePointer);
        nativePointer = 0;
        super.finalize();
    }

    private native void OnScanResult(long pointer,int callbackType,ScanResult result);

    private native void OnBatchScanResult(long pointer,List<ScanResult> result);

    private native void OnScanFailed(long pointer,int errorCode);
    private native void OnDestroy(long pointer);
}
