package com.ntu.group24.android.bluetooth;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.ntu.group24.android.R;

import java.util.ArrayList;

public class BluetoothDeviceAdapter extends ArrayAdapter<BluetoothDevice> {
    private final LayoutInflater mLayoutInflater;
    private final ArrayList<BluetoothDevice> mDevices;
    private final int mViewResourceId;

    public BluetoothDeviceAdapter(@NonNull Context context, int tvResourceId, @NonNull ArrayList<BluetoothDevice> devices) {
        super(context, tvResourceId, devices);
        this.mDevices = devices;
        this.mLayoutInflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        this.mViewResourceId = tvResourceId;
    }

    @NonNull
    @Override
    public View getView(int position, @Nullable View convertView, @NonNull ViewGroup parent) {
        if (convertView == null) {
            convertView = mLayoutInflater.inflate(mViewResourceId, parent, false);
        }

        BluetoothDevice device = mDevices.get(position);

        if (device != null) {
            TextView deviceName = convertView.findViewById(R.id.tvDeviceName);
            TextView deviceAddress = convertView.findViewById(R.id.tvDeviceAddress);

            // Handling Android 14 Security Requirements (C.2, C.8)
            // Use @SuppressLint > check permissions in the Fragment before scanning
            try {
                @SuppressLint("MissingPermission")
                String name = device.getName();
                if (deviceName != null) {
                    deviceName.setText(name != null ? name : getContext().getString(R.string.unknown_device));
                }
            } catch (SecurityException e) {
                if (deviceName != null) {
                    deviceName.setText(getContext().getString(R.string.error_permission_denied));
                }
            }

            if (deviceAddress != null) {
                deviceAddress.setText(device.getAddress());
            }
        }

        return convertView;
    }
}