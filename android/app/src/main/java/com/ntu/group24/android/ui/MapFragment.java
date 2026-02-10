package com.ntu.group24.android.ui;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.Toast;
import android.widget.TextView;
import android.content.BroadcastReceiver;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.Context;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import com.ntu.group24.android.R;
import com.ntu.group24.android.map.GridMap;
import com.ntu.group24.android.models.Obstacle;
import com.ntu.group24.android.utils.Constants;
import com.ntu.group24.android.models.RobotViewModel;

import java.util.Locale;

public class MapFragment extends Fragment {

    private GridMap gridMap;
    private TextView tvRobotStatus;
    private int currentTargetIndex = 0;

    // Resets ONLY when Task 1/Task 2 is sent (not every random TX)
    private final BroadcastReceiver taskResetReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded()) return;

            String msg = intent.getStringExtra("message");
            if (msg == null) return;

            if (msg.equals(Constants.START_EXPLORATION) || msg.equals(Constants.START_FASTEST_PATH)) {
                currentTargetIndex = 0;
                Log.d("MapFragment", "Task started: Target index reset to 0");
                broadcastRobotStatus("Ready to Start");
            }
        }
    };

    // C.9 receiver: RPi sends TARGET with obstacle no + imageId
    private final BroadcastReceiver targetDetectedReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded() || gridMap == null) return;

            int obstacleNo = intent.getIntExtra(Constants.EXTRA_OBSTACLE_NO, -1);
            int imageId = intent.getIntExtra(Constants.EXTRA_IMAGE_ID, -1);

            if (obstacleNo <= 0 || imageId <= 0) {
                Log.d("MapFragment", "Invalid TARGET payload: obstacleNo=" + obstacleNo + ", imageId=" + imageId);
                return;
            }

            Log.d("MapFragment", "TARGET detected: obstacleNo=" + obstacleNo + ", imageId=" + imageId);

            // Update the obstacle on the grid to show the imageId
            boolean updated = updateObstacleImageId(obstacleNo, imageId);

            if (!updated) {
                Log.d("MapFragment", "No matching obstacle found for obstacleNo=" + obstacleNo);
            }

            // Optional status update (C.4)
            // Example: "Finding Target 2" etc.
            // If obstacleNo is the "target index", you can keep your existing logic.
            // If obstacleNo is the obstacle's ID, then don't use it as target count.
        }
    };

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_map, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        RobotViewModel robotViewModel  = new ViewModelProvider(requireActivity()).get(RobotViewModel.class);

        // Initialise Map and Set Robot Controls
        gridMap = view.findViewById(R.id.gridMap);
        EditText etX = view.findViewById(R.id.etRobotX);
        EditText etY = view.findViewById(R.id.etRobotY);
        Button btnSet = view.findViewById(R.id.btnSetRobot);
        tvRobotStatus = view.findViewById(R.id.tvRobotStatus);

        // Initialise status for robot position
        tvRobotStatus.setText(getString(R.string.robot_status_format, 3, 3, "N"));

        // Observe changes from control fragment
        robotViewModel.getMoveRequest().observe(getViewLifecycleOwner(), direction -> {
            if (direction != null && gridMap != null) {
                gridMap.moveRobotManually(direction);
                robotViewModel.requestMovement(null);
            }
        });

        robotViewModel.getIncomingCommand().observe(getViewLifecycleOwner(), command -> {
            if (command != null && gridMap != null) {
                handleIncomingCommand(command);
            }
        });

        gridMap.setOnRobotMovedListener((x, y, direction) -> {
            if (isAdded()) {
                // x and y are the Anchor (Bottom-Left)
                int trX = x + 2;
                int trY = y + 2;

                // Update Tablet UI (Show Top-Right to user)
                String status = getString(R.string.robot_status_format, trX, trY, direction);
                tvRobotStatus.setText(status);

                // Sync AMD tool (Send Top-Right)
                MainActivity activity = (MainActivity) getActivity();
                if (activity != null && activity.getBluetoothService() != null) {
                    String syncCommand = String.format(Locale.US, "ROBOT,%d,%d,%s\n", trX, trY, direction);
                    activity.getBluetoothService().write(syncCommand);
                }
            }
        });

        btnSet.setOnClickListener(v -> {
            String xStr = etX.getText().toString();
            String yStr = etY.getText().toString();

            if (!xStr.isEmpty() && !yStr.isEmpty()) {
                try {
                    int trX = Integer.parseInt(xStr);
                    int trY = Integer.parseInt(yStr);

                    // Apply to local GridMap
                    String internalCmd = String.format(Locale.US, "ROBOT,%d,%d,N", trX, trY);
                    gridMap.applyCommand(internalCmd);

                    Toast.makeText(getContext(), "Robot set to TR: " + trX + "," + trY, Toast.LENGTH_SHORT).show();

                } catch (NumberFormatException e) {
                    Toast.makeText(getContext(), "Invalid numbers", Toast.LENGTH_SHORT).show();
                }
            }
        });

        // Tap empty cell to add obstacle (C.6)
        if (gridMap != null) {
            gridMap.setOnCellTapListener((x, y) -> {
                boolean inStartZone = (x >= 0 && x <= 3) && (y >= 0 && y <= 3);
                if (inStartZone) {
                    Toast.makeText(requireContext(), R.string.msg_start_zone_reserved, Toast.LENGTH_SHORT).show();
                } else {
                    showAddObstacleDialog(x, y);
                }
            });
        }
    }

    private void showAddObstacleDialog(int x0, int y0) {
        View dialogView = LayoutInflater.from(requireContext()).inflate(R.layout.dialog_add_obstacle, null);
        EditText idInput = dialogView.findViewById(R.id.inputObstacleId);
        Spinner faceSpinner = dialogView.findViewById(R.id.spinnerFace);

        ArrayAdapter<String> adapter = new ArrayAdapter<>(requireContext(),
                android.R.layout.simple_spinner_dropdown_item, new String[]{"N","E","S","W"});
        faceSpinner.setAdapter(adapter);

        new AlertDialog.Builder(requireContext())
                .setTitle("Add Obstacle")
                .setView(dialogView)
                .setPositiveButton("Add", (d, which) -> {
                    String idStr = idInput.getText().toString().trim();
                    if (idStr.isEmpty()) return;

                    int id = Integer.parseInt(idStr);
                    String faceStr = (String) faceSpinner.getSelectedItem();
                    Obstacle.Dir face = parseDir(faceStr);

                    // Update UI
                    gridMap.upsertObstacle(id, x0, y0, face);

                    // Send coordinates to RPi via Bluetooth (C.6, C.7)
                    MainActivity activity = (MainActivity) getActivity();
                    if (activity != null && activity.getBluetoothService() != null) {
                        String msg = String.format(Locale.US, Constants.OBSTACLE_ADD, id, x0, y0, faceStr);
                        activity.getBluetoothService().write(msg);
                    }
                })
                .setNegativeButton("Cancel", null).show();
    }

    private Obstacle.Dir parseDir(String s) {
        switch (s) {
            case "E": return Obstacle.Dir.E;
            case "S": return Obstacle.Dir.S;
            case "W": return Obstacle.Dir.W;
            default: return Obstacle.Dir.N;
        }
    }

    // C.9 core: update obstacle with imageId
    private boolean updateObstacleImageId(int obstacleNo, int imageId) {
        if (gridMap == null) return false;

        try {
            for (Obstacle o : gridMap.getObstacles().values()) {
                if (o != null && o.getId() == obstacleNo) {
                    o.setTargetId(imageId);   // <-- correct for your Obstacle.java
                    gridMap.invalidate();      // redraw
                    return true;
                }
            }
        } catch (Exception e) {
            Log.e("MapFragment", "updateObstacleImageId failed", e);
        }
        return false;
    }


    public void handleIncomingCommand(String command) {
        if (gridMap == null || command == null){
            return;
        }

        // Update map display
        gridMap.applyCommand(command);

        // Your existing status logic (C.4)
        if (command.startsWith("TARGET")) {
            String[] parts = command.split(",");
            try {
                int detectedTargetNo = Integer.parseInt(parts[1]);
                if (detectedTargetNo > currentTargetIndex) {
                    currentTargetIndex = detectedTargetNo;

                    int totalObstacles = gridMap.getObstacles().size();
                    if (currentTargetIndex < totalObstacles) {
                        broadcastRobotStatus("Looking for Target " + (currentTargetIndex + 1));
                    } else {
                        broadcastRobotStatus("All " + totalObstacles + " Targets Found - Task Complete");
                    }
                }
            } catch (Exception e) {
                currentTargetIndex++;
            }
        }
    }

    @Override
    public void onStart() {
        super.onStart();

        LocalBroadcastManager lbm = LocalBroadcastManager.getInstance(requireContext());

        // Task reset when Task 1/Task 2 is sent
        lbm.registerReceiver(taskResetReceiver, new IntentFilter(Constants.INTENT_MESSAGE_SENT));

        // C.9: target detected from BluetoothService
        lbm.registerReceiver(targetDetectedReceiver, new IntentFilter(Constants.INTENT_TARGET_DETECTED));
    }

    @Override
    public void onStop() {
        super.onStop();

        LocalBroadcastManager lbm = LocalBroadcastManager.getInstance(requireContext());
        lbm.unregisterReceiver(taskResetReceiver);
        lbm.unregisterReceiver(targetDetectedReceiver);
    }

    private void broadcastRobotStatus(String status) {
        if (!isAdded()) return;
        Intent intent = new Intent(Constants.INTENT_ROBOT_ACTIVITY_STATUS);
        intent.putExtra("message", status);
        LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(intent);
    }
}
