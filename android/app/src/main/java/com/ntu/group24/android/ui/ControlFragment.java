package com.ntu.group24.android.ui;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.ntu.group24.android.R;
import com.ntu.group24.android.utils.Constants;

public class ControlFragment extends Fragment {
    private static final String TAG = "ControlFragment";
    public ControlFragment() {
        // Empty public constructor
    }

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_control, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        // 1. Link Movement Buttons (C.3)
        Button btnForward = view.findViewById(R.id.btnForward);
        Button btnBack = view.findViewById(R.id.btnBack);
        Button btnLeft = view.findViewById(R.id.btnLeft);
        Button btnRight = view.findViewById(R.id.btnRight);

        // 2. Link Task Buttons
        Button btnTask1 = view.findViewById(R.id.btnTask1);
        Button btnTask2 = view.findViewById(R.id.btnTask2);

        // 3. Link Testing/Reset Buttons (C.6, C.8, C.10)
        Button btnReset = view.findViewById(R.id.btnReset);
        Button btnTestMove = view.findViewById(R.id.btnTestMove);

        btnForward.setOnClickListener(v -> sendCommand(Constants.MOVE_FORWARD));
        btnBack.setOnClickListener(v -> sendCommand(Constants.MOVE_BACKWARD));
        btnLeft.setOnClickListener(v -> sendCommand(Constants.TURN_LEFT));
        btnRight.setOnClickListener(v -> sendCommand(Constants.TURN_RIGHT));

        btnTask1.setOnClickListener(v -> sendCommand(Constants.START_EXPLORATION));
        btnTask2.setOnClickListener(v -> sendCommand(Constants.START_FASTEST_PATH));

        // Reset logic (C.6, C.8)
        btnReset.setOnClickListener(v -> {
            // Tell the GridMap to clear itself locally via the Broadcast system
            Intent clearIntent = new Intent(Constants.INTENT_MESSAGE_RECEIVED);
            clearIntent.putExtra("message", "CLEAR");
            LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(clearIntent);

            // Tell the RPi/Algorithm to reset their state
            sendCommand("RESET");
            Toast.makeText(requireContext(), "Map and Algorithm Reset", Toast.LENGTH_SHORT).show();
        });

        // Local test logic (C.10 Simulation)
        btnTestMove.setOnClickListener(v -> {
            // Simulate an incoming message from the RPi
            // MainActivity will catch this and update the GridMap icon
            Intent intent = new Intent(Constants.INTENT_MESSAGE_RECEIVED);
            intent.putExtra("message", "ROBOT,10,10,E");
            LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(intent);

            Toast.makeText(requireContext(), "Simulating ROBOT at (10,10) East", Toast.LENGTH_SHORT).show();
        });
    }

     // Helper to send commands via Bluetooth and log to the Comms tab (C.1, C.4)
    private void sendCommand(String cmd) {
        // 1. Log locally to the Communications Fragment (TX)
        // Sends internal signal so chat logs shows '[TX] f' in blue
        Intent i = new Intent(Constants.INTENT_MESSAGE_SENT);
        i.putExtra("message", cmd);
        LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(i);

        // 2. Send via Bluetooth Service (C.1)
        MainActivity activity = (MainActivity) getActivity();
        if (activity != null && activity.getBluetoothService() != null) {
            activity.getBluetoothService().write(cmd);
            // Visual feedback for operator (C.3)
            Toast.makeText(requireContext(), "Sent: " + cmd, Toast.LENGTH_SHORT).show();
            Log.d(TAG, "Sent via BT: " + cmd);
        } else {
            // Inform user when connection is dead (C.8)
            Toast.makeText(requireContext(), "Bluetooth not ready", Toast.LENGTH_SHORT).show();
        }
    }
}