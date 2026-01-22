package com.ntu.group24.android.ui;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ListView;
import android.widget.Toast;

import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;

import com.ntu.group24.android.R;
import com.ntu.group24.android.bluetooth.BluetoothDeviceAdapter;

import java.util.ArrayList;

public class BluetoothFragment extends Fragment {
    private static final String TAG = "BluetoothFragment";

    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothDeviceAdapter mDeviceAdapter;
    private ArrayList<BluetoothDevice> mNewDevicesList = new ArrayList<>();
    private ListView lvNewDevices;

    private final BroadcastReceiver mBroadcastReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                try {
                    BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);

                    if (device != null) {
                        @android.annotation.SuppressLint("MissingPermission")
                        String deviceName = device.getName();
                        String deviceAddress = device.getAddress();

                        Log.d(TAG, "Device found: " + deviceName + " [" + deviceAddress + "]");

                        // Update list and UI (C.2)
                        if (!mNewDevicesList.contains(device)) {
                            mNewDevicesList.add(device);
                            if (mDeviceAdapter != null) {
                                mDeviceAdapter.notifyDataSetChanged();
                            }
                        }
                    }
                } catch (SecurityException e) {
                    Log.e(TAG, "Permission missing for device name", e);
                }
            }
        }
    };

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.fragment_bluetooth, container, false);

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        lvNewDevices = root.findViewById(R.id.lvDevices);
        mNewDevicesList = new ArrayList<>();

        Button btnScan = root.findViewById(R.id.btnScan);
        btnScan.setOnClickListener(v -> startScanning());

        return root;
    }

    private void startScanning() {
        // Check if permissions are granted before calling bluetooth methods (C.8)
        if (ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            Toast.makeText(getContext(), "Scan permission not granted!", Toast.LENGTH_SHORT).show();
            return;
        }

        try {
            if (mBluetoothAdapter.isDiscovering()) {
                mBluetoothAdapter.cancelDiscovery();
            }

            mNewDevicesList.clear();
            mDeviceAdapter = new BluetoothDeviceAdapter(requireContext(), R.layout.item_bluetooth_device, mNewDevicesList);
            lvNewDevices.setAdapter(mDeviceAdapter);

            mBluetoothAdapter.startDiscovery();

            IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
            requireActivity().registerReceiver(mBroadcastReceiver, filter);

            Toast.makeText(getContext(), "Scanning...", Toast.LENGTH_SHORT).show();
        } catch (SecurityException e) {
            Log.e(TAG, "SecurityException during scan", e);
        }
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        // Safely unregister (C.8)
        try {
            if (getActivity() != null) {
                getActivity().unregisterReceiver(mBroadcastReceiver);
            }
        } catch (IllegalArgumentException e) {
            Log.d(TAG, "Receiver already unregistered");
        }
    }
}