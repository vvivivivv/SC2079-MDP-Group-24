package com.ntu.group24.android.bluetooth;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.graphics.Color;
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

                // Highlights our RPi (C.2)
                if (name != null && name.contains("24")) {
                    deviceName.setText(name);
                    deviceName.setTextColor(Color.parseColor("#2E7D32"));
                } else if (name != null) {
                    deviceName.setText(name);
                    deviceName.setTextColor(Color.BLACK);
                } else {
                    deviceName.setText(getContext().getString(R.string.unknown_device));
                    deviceName.setTextColor(Color.GRAY);
                }

                // Show bond state (C.8)
                @SuppressLint("MissingPermission")
                int bondState = device.getBondState();
                String address = device.getAddress();
                if (bondState == BluetoothDevice.BOND_BONDED) {
                    deviceAddress.setText(String.format("%s (PAIRED)", address));
                } else {
                    deviceAddress.setText(address);
                }

            } catch (SecurityException e) {
                deviceName.setText(getContext().getString(R.string.error_permission_denied));
            }
        }
        return convertView;
    }
}