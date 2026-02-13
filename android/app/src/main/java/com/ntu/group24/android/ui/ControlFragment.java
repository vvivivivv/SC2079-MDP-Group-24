package com.ntu.group24.android.ui;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Toast;
import android.content.Intent;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import androidx.lifecycle.ViewModelProvider;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.ntu.group24.android.R;
import com.ntu.group24.android.utils.Constants;
import com.ntu.group24.android.models.RobotViewModel;

public class ControlFragment extends Fragment {
    private RobotViewModel robotViewModel;

    public ControlFragment() {
        // Empty public constructor
    }

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container,
                             Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_control, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view,
                              @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        robotViewModel = new ViewModelProvider(requireActivity()).get(RobotViewModel.class);

        Button btnForward = view.findViewById(R.id.btnForward);
        Button btnBack = view.findViewById(R.id.btnBack);
        Button btnLeft = view.findViewById(R.id.btnLeft);
        Button btnRight = view.findViewById(R.id.btnRight);
        Button btnTask1 = view.findViewById(R.id.btnTask1);
        Button btnTask2 = view.findViewById(R.id.btnTask2);

        // Movement controls (C.3)
        btnForward.setOnClickListener(v -> moveRobot("FORWARD"));
        btnBack.setOnClickListener(v -> moveRobot("BACKWARD"));
        btnLeft.setOnClickListener(v -> moveRobot("LEFT"));
        btnRight.setOnClickListener(v -> moveRobot("RIGHT"));

        // Task controls
        btnTask1.setOnClickListener(v -> sendCommand(Constants.START_EXPLORATION));
        btnTask2.setOnClickListener(v -> sendCommand(Constants.START_FASTEST_PATH));
    }

    private void moveRobot(String direction) {
        MainActivity activity = (MainActivity) getActivity();
        String cmd = "";

        // Map to specific protocols required by the AMD tool/RPi
        switch (direction) {
            case "FORWARD":  cmd = "f";  break;
            case "BACKWARD": cmd = "b";  break;
            case "LEFT":     cmd = "tl"; break;
            case "RIGHT":    cmd = "tr"; break;
        }

        // Send the command (f, b, tl, tr) to Bluetooth
        if (activity != null && activity.getBluetoothService() != null && !cmd.isEmpty()) {
            activity.getBluetoothService().write(cmd);
        }

        // Sync tablet UI via ViewModel, trigger observer in map fragment
        robotViewModel.requestMovement(direction);
    }

    private void sendCommand(String cmd) {
        // Always log it to COMMS (even if BT not connected)
        MainActivity activity = (MainActivity) getActivity();
        if (activity == null || activity.getBluetoothService() == null) {
            Toast.makeText(requireContext(), "Bluetooth not ready", Toast.LENGTH_SHORT).show();
            return;
        }

        // Send to RPi
        activity.getBluetoothService().write(cmd);

        // Broadcast status for task buttons (C.4)
        if (cmd.equals(Constants.START_EXPLORATION) || cmd.equals(Constants.START_FASTEST_PATH)) {
            Intent statusIntent = new Intent(Constants.INTENT_ROBOT_ACTIVITY_STATUS);
            statusIntent.putExtra("message", "Ready to Start: Looking for Target 1");
            LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(statusIntent);
        }

        Toast.makeText(requireContext(), "Sent: " + cmd, Toast.LENGTH_SHORT).show();
    }
    
}