package com.ntu.group24.android.ui;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Toast;
import android.content.Intent;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.ntu.group24.android.R;
import com.ntu.group24.android.utils.Constants;
// import com.ntu.group24.android.bluetooth.BluetoothService;

import java.nio.charset.StandardCharsets;

public class ControlFragment extends Fragment {

    // Uncomment when BluetoothService is ready
    // private BluetoothService btService;

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

        // Uncomment when MainActivity exposes BluetoothService
        /*
        if (getActivity() instanceof MainActivity) {
            btService = ((MainActivity) getActivity()).getBluetoothService();
        }
        */

        Button btnForward = view.findViewById(R.id.btnForward);
        Button btnBack = view.findViewById(R.id.btnBack);
        Button btnLeft = view.findViewById(R.id.btnLeft);
        Button btnRight = view.findViewById(R.id.btnRight);
        Button btnTask1 = view.findViewById(R.id.btnTask1);
        Button btnTask2 = view.findViewById(R.id.btnTask2);

        // Movement controls (C.3)
        btnForward.setOnClickListener(v -> sendCommand(Constants.MOVE_FORWARD));
        btnBack.setOnClickListener(v -> sendCommand(Constants.MOVE_BACKWARD));
        btnLeft.setOnClickListener(v -> sendCommand(Constants.TURN_LEFT));
        btnRight.setOnClickListener(v -> sendCommand(Constants.TURN_RIGHT));

        // Task controls
        btnTask1.setOnClickListener(v -> sendCommand(Constants.START_EXPLORATION));
        btnTask2.setOnClickListener(v -> sendCommand(Constants.START_FASTEST_PATH));

    }
    private void sendCommand(String cmd) {
        // Always log it to COMMS (even if BT not connected)
        Intent i = new Intent(Constants.INTENT_MESSAGE_SENT);
        i.putExtra("message", cmd);
        LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(i);

        MainActivity activity = (MainActivity) getActivity();
        if (activity == null || activity.getBluetoothService() == null) {
            Toast.makeText(requireContext(), "Bluetooth not ready", Toast.LENGTH_SHORT).show();
            return;
        }

        // Send to RPi
        activity.getBluetoothService().write(cmd);

        Toast.makeText(requireContext(), "Sent: " + cmd, Toast.LENGTH_SHORT).show();
    }


}
