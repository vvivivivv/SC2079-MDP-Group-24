package com.ntu.group24.android.ui;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.ntu.group24.android.R;
import com.ntu.group24.android.bluetooth.BluetoothDeviceAdapter;
import com.ntu.group24.android.utils.Constants;

import java.util.ArrayList;

public class BluetoothFragment extends Fragment {
    private static final String TAG = "BluetoothFragment";

    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothDeviceAdapter mDeviceAdapter;
    private final ArrayList<BluetoothDevice> mNewDevicesList = new ArrayList<>();

    private ListView lvNewDevices;
    private TextView tvMessageLog;
    private TextView tvConnectionStatus;

    private boolean isRetrying = false;
    private final Handler reconnectionHandler = new Handler();

    private boolean scanReceiverRegistered = false;

    private SharedPreferences prefs;

    private final BroadcastReceiver mScanReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!BluetoothDevice.ACTION_FOUND.equals(intent.getAction())) return;

            try {
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (device != null && !mNewDevicesList.contains(device)) {
                    mNewDevicesList.add(device);
                    if (mDeviceAdapter != null) mDeviceAdapter.notifyDataSetChanged();
                }
            } catch (SecurityException e) {
                Log.e(TAG, "Discovery permission error", e);
            }
        }
    };

    private final BroadcastReceiver mDataReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded() || getContext() == null) return;

            String action = intent.getAction();
            if (action == null) return;

            if (Constants.INTENT_MESSAGE_RECEIVED.equals(action)) {
                String message = intent.getStringExtra("message");

                // keep your “don’t spam robot coords” rule
                if (message != null && !message.startsWith(Constants.HEADER_ROBOT)) {
                    tvMessageLog.append(getString(R.string.robot_log_format, message));
                    int scrollAmount = tvMessageLog.getLayout() == null ? 0
                            : tvMessageLog.getLayout().getLineTop(tvMessageLog.getLineCount()) - tvMessageLog.getHeight();
                    if (scrollAmount > 0) tvMessageLog.scrollTo(0, scrollAmount);
                }

            } else if (Constants.INTENT_CONNECTION_STATUS.equals(action)) {
                String status = intent.getStringExtra("status");
                if (status == null) return;

                tvConnectionStatus.setText(getString(R.string.status_format, status));

                // If your service broadcasts "Connected"/"Disconnected"/"Connecting..."
                if (status.equalsIgnoreCase("Disconnected")) {
                    if (!isRetrying) {
                        isRetrying = true;
                        attemptAutoReconnect();
                    }
                } else if (status.equalsIgnoreCase("Connected")) {
                    isRetrying = false;
                    reconnectionHandler.removeCallbacksAndMessages(null);
                }
            }
        }
    };

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.fragment_bluetooth, container, false);

        prefs = requireContext().getSharedPreferences("app_prefs", Context.MODE_PRIVATE);

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        lvNewDevices = root.findViewById(R.id.lvDevices);
        tvMessageLog = root.findViewById(R.id.tvMessageLog);
        tvConnectionStatus = root.findViewById(R.id.tvConnectionStatus);

        tvMessageLog.setMovementMethod(new ScrollingMovementMethod());

        Button btnScan = root.findViewById(R.id.btnScan);
        btnScan.setOnClickListener(v -> startScanning());

        // Re-purpose "Listen" button to "Reconnect last device"
        Button btnListen = root.findViewById(R.id.btnListen);
        btnListen.setOnClickListener(v -> reconnectLastDevice());

        // Tap a scanned device to connect
        lvNewDevices.setOnItemClickListener(new android.widget.AdapterView.OnItemClickListener() {
            @SuppressLint("MissingPermission")
            @Override
            public void onItemClick(android.widget.AdapterView<?> parent, View view, int position, long id) {

                // Android 12+ requires BLUETOOTH_SCAN for these calls
                if (ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_SCAN)
                        == PackageManager.PERMISSION_GRANTED) {
                    if (mBluetoothAdapter != null && mBluetoothAdapter.isDiscovering()) {
                        mBluetoothAdapter.cancelDiscovery();
                    }
                }

                BluetoothDevice selectedDevice = mNewDevicesList.get(position);

                // save MAC for reconnect
                prefs.edit().putString(Constants.PREF_LAST_CONNECTED_DEVICE, selectedDevice.getAddress()).apply();

                String deviceName;
                try {
                    deviceName = selectedDevice.getName();
                } catch (SecurityException e) {
                    deviceName = null;
                }
                if (deviceName == null) deviceName = selectedDevice.getAddress();

                MainActivity activity = (MainActivity) getActivity();
                if (activity != null && activity.getBluetoothService() != null) {
                    activity.getBluetoothService().connect(selectedDevice);
                    Toast.makeText(getContext(), "Connecting to " + deviceName, Toast.LENGTH_SHORT).show();
                } else {
                    Toast.makeText(getContext(), "BluetoothService not ready", Toast.LENGTH_SHORT).show();
                    Log.e(TAG, "BluetoothService not initialized!");
                }
            }
        });


        // init adapter once
        mDeviceAdapter = new BluetoothDeviceAdapter(requireContext(), R.layout.item_bluetooth_device, mNewDevicesList);
        lvNewDevices.setAdapter(mDeviceAdapter);

        return root;
    }

    private void reconnectLastDevice() {
        String lastMac = prefs.getString(Constants.PREF_LAST_CONNECTED_DEVICE, null);
        if (lastMac == null) {
            Toast.makeText(getContext(), "No saved device to reconnect", Toast.LENGTH_SHORT).show();
            return;
        }

        if (mBluetoothAdapter == null) {
            Toast.makeText(getContext(), "Bluetooth not supported", Toast.LENGTH_SHORT).show();
            return;
        }

        try {
            BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(lastMac);

            MainActivity activity = (MainActivity) getActivity();
            if (activity != null && activity.getBluetoothService() != null) {
                activity.getBluetoothService().connect(device);
                Toast.makeText(getContext(), "Reconnecting to " + lastMac, Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(getContext(), "BluetoothService not ready", Toast.LENGTH_SHORT).show();
            }
        } catch (IllegalArgumentException e) {
            Toast.makeText(getContext(), "Saved MAC invalid", Toast.LENGTH_SHORT).show();
        }
    }

    private void attemptAutoReconnect() {
        if (!isAdded()) return;

        String lastMac = prefs.getString(Constants.PREF_LAST_CONNECTED_DEVICE, null);
        if (lastMac == null) {
            isRetrying = false;
            return;
        }

        Toast.makeText(getContext(), R.string.msg_reconnecting, Toast.LENGTH_SHORT).show();

        reconnectionHandler.postDelayed(() -> {
            if (!isRetrying || !isAdded()) return;
            reconnectLastDevice();
        }, 2000);
    }

    @SuppressLint("MissingPermission")
    private void startScanning() {
        if (mBluetoothAdapter == null) {
            Toast.makeText(getContext(), "Bluetooth not supported", Toast.LENGTH_SHORT).show();
            return;
        }

        // need BLUETOOTH_SCAN on Android 12+
        if (ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_SCAN)
                != PackageManager.PERMISSION_GRANTED) {
            Toast.makeText(getContext(), "Bluetooth scan permission missing", Toast.LENGTH_SHORT).show();
            return;
        }

        if (mBluetoothAdapter.isDiscovering()) mBluetoothAdapter.cancelDiscovery();

        mNewDevicesList.clear();
        if (mDeviceAdapter != null) mDeviceAdapter.notifyDataSetChanged();

        if (!scanReceiverRegistered) {
            requireActivity().registerReceiver(mScanReceiver, new IntentFilter(BluetoothDevice.ACTION_FOUND));
            scanReceiverRegistered = true;
        }

        boolean ok = mBluetoothAdapter.startDiscovery();
        if (!ok) Toast.makeText(getContext(), "Discovery failed to start", Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onResume() {
        super.onResume();
        IntentFilter filter = new IntentFilter();
        filter.addAction(Constants.INTENT_MESSAGE_RECEIVED);
        filter.addAction(Constants.INTENT_CONNECTION_STATUS);
        LocalBroadcastManager.getInstance(requireContext()).registerReceiver(mDataReceiver, filter);
    }

    @Override
    public void onPause() {
        super.onPause();
        LocalBroadcastManager.getInstance(requireContext()).unregisterReceiver(mDataReceiver);
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (scanReceiverRegistered) {
            try {
                requireActivity().unregisterReceiver(mScanReceiver);
            } catch (Exception e) {
                Log.d(TAG, "Unregister scan receiver failed");
            }
            scanReceiverRegistered = false;
        }
    }
}
