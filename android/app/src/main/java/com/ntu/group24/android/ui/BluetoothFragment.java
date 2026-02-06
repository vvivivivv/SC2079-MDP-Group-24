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

    private final BroadcastReceiver mScanReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (BluetoothDevice.ACTION_FOUND.equals(intent.getAction())) {
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
        }
    };

    private final BroadcastReceiver mDataReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (!isAdded() || getContext() == null) return;
            if (Constants.INTENT_MESSAGE_RECEIVED.equals(action)) {
                String message = intent.getStringExtra("message");

                // Selective info (don't log robot coordinates) (C.4)
                if (message != null && !message.startsWith(Constants.HEADER_ROBOT)) {
                    tvMessageLog.append(getString(R.string.robot_log_format, message));
                //if (message != null) {
                    //tvMessageLog.append("[RX] " + message + "\n");

                    final int scrollAmount = tvMessageLog.getLayout().getLineTop(tvMessageLog.getLineCount()) - tvMessageLog.getHeight();
                    if (scrollAmount > 0) tvMessageLog.scrollTo(0, scrollAmount);
                }

            } else if (Constants.INTENT_CONNECTION_STATUS.equals(action)) {
                String status = intent.getStringExtra("status");

                if (status != null) {
                    tvConnectionStatus.setText(getString(R.string.status_format, status));
                    String lowerStatus = status.toLowerCase();
                    if (lowerStatus.equals("connected")) {
                        tvConnectionStatus.setTextColor(ContextCompat.getColor(requireContext(), android.R.color.holo_green_dark));
                        isRetrying = false;
                        reconnectionHandler.removeCallbacksAndMessages(null);
                    } else if (lowerStatus.contains("connecting") || lowerStatus.contains("listening")) {
                        tvConnectionStatus.setTextColor(ContextCompat.getColor(requireContext(), android.R.color.holo_orange_dark));
                    } else {
                        tvConnectionStatus.setTextColor(ContextCompat.getColor(requireContext(), android.R.color.holo_red_dark));
                        if (!isRetrying){
                            isRetrying = true;
                            attemptAutoReconnect();
                        }
                    }
                    // Auto-reconnect logic (C.8)
                   /* if (getString(R.string.state_disconnected).equals(status) && !isRetrying) {
                        isRetrying = true;
                        attemptAutoReconnect();
                    } else if (getString(R.string.state_connected).equals(status)) {
                        isRetrying = false;
                        reconnectionHandler.removeCallbacksAndMessages(null);
                    }*/
                }
            }
        }
    };

    @Override
    public void onStart() {
        super.onStart();
        IntentFilter filter = new IntentFilter();
        filter.addAction(Constants.INTENT_MESSAGE_RECEIVED);
        filter.addAction(Constants.INTENT_CONNECTION_STATUS);
        LocalBroadcastManager.getInstance(requireContext()).registerReceiver(mDataReceiver, filter);
        Log.d(TAG, "Receiver Registered in onStart");
    }

    @Override
    public void onStop() {
        super.onStop();
        LocalBroadcastManager.getInstance(requireContext()).unregisterReceiver(mDataReceiver);
        Log.d(TAG, "Receiver Unregistered in onStop");
    }

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.fragment_bluetooth, container, false);

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        lvNewDevices = root.findViewById(R.id.lvDevices);
        tvMessageLog = root.findViewById(R.id.tvMessageLog);
        tvConnectionStatus = root.findViewById(R.id.tvConnectionStatus);

        tvMessageLog.setMovementMethod(new ScrollingMovementMethod());

        Button btnScan = root.findViewById(R.id.btnScan);
        btnScan.setOnClickListener(v -> startScanning());

        Button btnListen = root.findViewById(R.id.btnListen);

        btnListen.setOnClickListener(v -> {
            MainActivity activity = (MainActivity) getActivity();
            if (activity != null && activity.getBluetoothService() != null) {

                // Request Discoverability
                Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
                discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
                startActivity(discoverableIntent);

                // Start the AcceptThread manually
                activity.getBluetoothService().start();

                tvConnectionStatus.setText("Status: Listening...");
                Toast.makeText(getContext(), "Waiting for AMD Tool connection...", Toast.LENGTH_SHORT).show();
            }
        });

        lvNewDevices.setOnItemClickListener((parent, view, position, id) -> {
            // Explicitly check for BLUETOOTH_CONNECT permission
            if (androidx.core.content.ContextCompat.checkSelfPermission(requireContext(),
                    android.Manifest.permission.BLUETOOTH_CONNECT) != android.content.pm.PackageManager.PERMISSION_GRANTED
                    && android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
                Toast.makeText(getContext(), "Connect permission required", Toast.LENGTH_SHORT).show();
                return;
            }

            try {
                // Stop discovery
                if (mBluetoothAdapter.isDiscovering()) {
                    mBluetoothAdapter.cancelDiscovery();
                }

                // Get device
                BluetoothDevice device = mNewDevicesList.get(position);
                String deviceName = device.getName() != null ? device.getName() : "Unknown Device";
                Log.d(TAG, "Connecting to: " + deviceName + " [" + device.getAddress() + "]");

                // Initiate connection via Service
                MainActivity activity = (MainActivity) getActivity();
                if (activity != null && activity.getBluetoothService() != null) {
                    tvConnectionStatus.setText("Status: Connecting...");
                    activity.getBluetoothService().connect(device);
                }
            } catch (SecurityException e) {
                Log.e(TAG, "Permission denied during connection attempt", e);
            }
        });

        return root;
    }

    private void attemptAutoReconnect() {
        if (!isAdded()) return;
        Toast.makeText(getContext(), R.string.msg_reconnecting, Toast.LENGTH_SHORT).show();
        reconnectionHandler.postDelayed(() -> {
            if (isRetrying && isAdded()) {
                MainActivity activity = (MainActivity) getActivity();
                if (activity != null && activity.getBluetoothService() != null) {
                    activity.getBluetoothService().start();
                }
            }
        }, 5000);
    }

    @android.annotation.SuppressLint("MissingPermission")
    private void startScanning() {
        // Check if permissions are granted before calling bluetooth methods (C.8)
        if (ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        if (mBluetoothAdapter.isDiscovering()) mBluetoothAdapter.cancelDiscovery();
        mNewDevicesList.clear();
        mDeviceAdapter = new BluetoothDeviceAdapter(requireContext(), R.layout.item_bluetooth_device, mNewDevicesList);
        lvNewDevices.setAdapter(mDeviceAdapter);
        mBluetoothAdapter.startDiscovery();
        requireActivity().registerReceiver(mScanReceiver, new IntentFilter(BluetoothDevice.ACTION_FOUND));
    }

  /*  @Override
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
    }*/

    @Override
    public void onDestroy() {
        super.onDestroy();
        try {
            requireActivity().unregisterReceiver(mScanReceiver);
        } catch (Exception e) {
            Log.d(TAG, "Unregistering receiver failed");
        }
    }
}