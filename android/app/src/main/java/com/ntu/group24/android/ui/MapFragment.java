package com.ntu.group24.android.ui;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.EditText;
import android.widget.Toast;
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

import com.google.android.material.tabs.TabLayout;

import java.util.Locale;

public class MapFragment extends Fragment {

    private GridMap gridMap;
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

            boolean updated = updateObstacleImageId(obstacleNo, imageId);
            if (!updated) Log.d("MapFragment", "No matching obstacle found for obstacleNo=" + obstacleNo);
        }
    };

    // NEW: when GridMap renumbers/deletes/face-changes, do a full sync
    private final BroadcastReceiver fullSyncReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded() || gridMap == null) return;
            syncAllObstaclesToRpi();
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

        TabLayout subTabs = view.findViewById(R.id.map_sub_tabs);
        subTabs.addOnTabSelectedListener(new TabLayout.OnTabSelectedListener() {
            @Override
            public void onTabSelected(TabLayout.Tab tab) {
                Fragment selected = (tab.getPosition() == 0) ? new ControlFragment() : new CommunicationsFragment();
                getChildFragmentManager().beginTransaction()
                        .replace(R.id.map_bottom_container, selected)
                        .commit();
            }
            @Override public void onTabUnselected(TabLayout.Tab tab) {}
            @Override public void onTabReselected(TabLayout.Tab tab) {}
        });

        // Load default (Controls)
        getChildFragmentManager().beginTransaction()
                .replace(R.id.map_bottom_container, new ControlFragment())
                .commit();

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
                robotViewModel.setRobotStatus(status);

                // Sync AMD tool (Send Top-Right)
                MainActivity activity = (MainActivity) getActivity();
                if (activity != null && activity.getBluetoothService() != null) {
                    String syncCommand = String.format(Locale.US, "ROBOT,%d,%d,%s\n", trX, trY, direction);
                    activity.getBluetoothService().write(syncCommand);
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

        // keep hiding the id input (auto numbering)
        EditText idInput = dialogView.findViewById(R.id.inputObstacleId);
        if (idInput != null) idInput.setVisibility(View.GONE);

        // NEW: radio group instead of spinner
        android.widget.RadioGroup rgFace = dialogView.findViewById(R.id.rgFace);

        new AlertDialog.Builder(requireContext())
                .setTitle("Add Obstacle")
                .setView(dialogView)
                .setPositiveButton("Add", (d, which) -> {

                    int id = gridMap.getNextObstacleId();

                    // Default face
                    String faceStr = "N";
                    if (rgFace != null) {
                        int checkedId = rgFace.getCheckedRadioButtonId();
                        if (checkedId == R.id.rbE) faceStr = "E";
                        else if (checkedId == R.id.rbS) faceStr = "S";
                        else if (checkedId == R.id.rbW) faceStr = "W";
                    }

                    Obstacle.Dir face = parseDir(faceStr);

                    // Update UI
                    gridMap.upsertObstacle(id, x0, y0, face);

                    MainActivity activity = (MainActivity) getActivity();
                    if (activity != null && activity.getBluetoothService() != null) {
                        String msg = String.format(Locale.US, Constants.OBSTACLE_ADD, id, x0, y0, faceStr);
                        activity.getBluetoothService().write(msg);
                    }
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private Obstacle.Dir parseDir(String s) {
        switch (s) {
            case "E": return Obstacle.Dir.E;
            case "S": return Obstacle.Dir.S;
            case "W": return Obstacle.Dir.W;
            default: return Obstacle.Dir.N;
        }
    }

    // Update obstacle with imageId (C.9)
    private boolean updateObstacleImageId(int obstacleNo, int imageId) {
        if (gridMap == null) return false;

        try {
            for (Obstacle o : gridMap.getObstacles().values()) {
                if (o != null && o.getId() == obstacleNo) {
                    o.setTargetId(imageId);
                    gridMap.invalidate();
                    return true;
                }
            }
        } catch (Exception e) {
            Log.e("MapFragment", "updateObstacleImageId failed", e);
        }
        return false;
    }

    // NEW: full sync map to RPi after any add/remove/renumber/face change
    private void syncAllObstaclesToRpi() {
        if (!isAdded() || gridMap == null) return;

        MainActivity activity = (MainActivity) getActivity();
        if (activity == null || activity.getBluetoothService() == null) return;

        // Optional: if your RPi supports CLEAR
        activity.getBluetoothService().write("CLEAR");

        for (Obstacle o : gridMap.getObstacles().values()) {
            if (o == null) continue;

            String msg = String.format(Locale.US, Constants.OBSTACLE_ADD,
                    o.getId(), o.getX(), o.getY(), o.getFace().name());

            activity.getBluetoothService().write(msg);
        }
    }

    private void logComms(String message) {
        Intent i = new Intent(Constants.INTENT_MESSAGE_SENT);
        i.putExtra("message", message);
        LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(i);
    }


    public void handleIncomingCommand(String command) {
        if (gridMap == null || command == null){
            return;
        }

        // Update map display
        gridMap.applyCommand(command);

        // Update status messages (C.4)
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

        // Target detected from BluetoothService (C.9)
        lbm.registerReceiver(targetDetectedReceiver, new IntentFilter(Constants.INTENT_TARGET_DETECTED));

        // NEW: full sync signal from GridMap after delete/renumber/face change
        lbm.registerReceiver(fullSyncReceiver, new IntentFilter(Constants.INTENT_OBSTACLE_MAP_DIRTY));
    }

    @Override
    public void onStop() {
        super.onStop();

        LocalBroadcastManager lbm = LocalBroadcastManager.getInstance(requireContext());
        lbm.unregisterReceiver(taskResetReceiver);
        lbm.unregisterReceiver(targetDetectedReceiver);
        lbm.unregisterReceiver(fullSyncReceiver);
    }

    private void broadcastRobotStatus(String status) {
        if (!isAdded()) return;
        Intent intent = new Intent(Constants.INTENT_ROBOT_ACTIVITY_STATUS);
        intent.putExtra("message", status);
        LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(intent);
    }
}
