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
    private long exploreStartMs = 0L;
    private boolean exploreRunning = false;
    private boolean isMapDirty = false;
    private final android.os.Handler timerHandler = new android.os.Handler(android.os.Looper.getMainLooper());
    private final Runnable timerTick = new Runnable() {
        @Override
        public void run() {
            if (!exploreRunning) return;

            long elapsed = System.currentTimeMillis() - exploreStartMs;
            broadcastTimer(elapsed, true);

            timerHandler.postDelayed(this, 250); // update 4x/sec
        }
    };
    // Resets ONLY when Task 1/Task 2 is sent (not every random TX)
    private final BroadcastReceiver taskResetReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded()) return;

            String msg = intent.getStringExtra("message");
            if (msg == null) return;

            if (msg.equals(Constants.START_EXPLORATION) || msg.equals(Constants.START_FASTEST_PATH)) {
                startExploreTimer();
                currentTargetIndex = 0;
                Log.d("MapFragment", "Task started: Target index reset to 0");
            }
        }
    };
    //timer
    private final BroadcastReceiver endReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            stopExploreTimer();
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

    // Now: only mark dirty + log pending change (no auto sync)
    private final BroadcastReceiver dirtyReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded()) return;

            isMapDirty = true;

            String msg = intent.getStringExtra("message");
            if (msg != null) {
                logComms("[PENDING] " + msg);
            }
        }
    };

    private final BroadcastReceiver incomingMessageReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded()) return;
            String msg = intent.getStringExtra("message");
            if (msg != null) {
                handleIncomingCommand(msg);
            }
        }
    };

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        return inflater.inflate(R.layout.fragment_map, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        RobotViewModel robotViewModel = new ViewModelProvider(requireActivity()).get(RobotViewModel.class);

        // Initialise Map and Set Robot Controls
        gridMap = view.findViewById(R.id.gridMap);

        // NEW: Sync button (add this in your fragment_map.xml)
        View btnSync = view.findViewById(R.id.btnSyncObstacles);
        if (btnSync != null) {
            btnSync.setOnClickListener(v -> {
                syncAllObstaclesToRpi();
                isMapDirty = false;
                Toast.makeText(requireContext(), "Obstacles synced to RPi", Toast.LENGTH_SHORT).show();
            });
        }

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
            if (!isAdded()) return;

            int trX = x + 2;
            int trY = y + 2;

            String status = getString(R.string.robot_status_format, trX, trY, direction);
            robotViewModel.setRobotStatus(status);

            isMapDirty = true;
            broadcastRobotStatus("Robot updated (Unsynced)");
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
    private final BroadcastReceiver syncRequestReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded() || gridMap == null) return;
            syncAllObstaclesToRpi();   // send now
        }
    };

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
                    if (gridMap == null) return;

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

                    isMapDirty = true;
                    logComms("[PENDING] ADD," + id + "," + x0 + "," + y0 + "," + faceStr);
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

    // ONLY sends when Sync button pressed
    private void syncAllObstaclesToRpi() {
        if (!isAdded() || gridMap == null) return;

        MainActivity activity = (MainActivity) getActivity();
        if (activity == null || activity.getBluetoothService() == null) {
            Toast.makeText(requireContext(), "Bluetooth not ready", Toast.LENGTH_SHORT).show();
            return;
        }

        final java.util.List<String> syncQueue = new java.util.ArrayList<>();

        // 1. Clear RPi memory
        syncQueue.add("CLEAR");

        // 2. Add robot state with latest face
        int trX = gridMap.getRobot().getX() + 2;
        int trY = gridMap.getRobot().getY() + 2;
        String dir = gridMap.getRobot().getDirection();
        syncQueue.add(String.format(Locale.US, "ROBOT,%d,%d,%s", trX, trY, dir));

        // 3. Send all obstacles
        for (Obstacle o : gridMap.getObstacles().values()) {
            if (o == null) continue;
            syncQueue.add(String.format(Locale.US, Constants.OBSTACLE_ADD,
                    o.getId(), o.getX(), o.getY(), o.getFace().name()));
        }

        Toast.makeText(requireContext(), "Syncing map (0.5s delay)...", Toast.LENGTH_SHORT).show();
        processSyncQueue(syncQueue, activity);
    }

    private void processSyncQueue(final java.util.List<String> queue, final MainActivity activity) {
        if (queue.isEmpty()) {
            isMapDirty = false;
            broadcastRobotStatus("Map Synced");
            Toast.makeText(requireContext(), "Map fully synced to RPi", Toast.LENGTH_SHORT).show();
            return;
        }

        String msg = queue.remove(0);
        activity.getBluetoothService().write(msg);

        // Schedule the next command in 500ms
        new android.os.Handler(android.os.Looper.getMainLooper()).postDelayed(new Runnable() {
            @Override
            public void run() {
                if (isAdded()) {
                    processSyncQueue(queue, activity);
                }
            }
        }, 500);
    }

    private void logComms(String message) {
        com.ntu.group24.android.utils.CommsLog.add(message);
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

        // Algo computation complete
        if (command.trim().equalsIgnoreCase(Constants.COMP_DONE)) {
            Log.d("MapFragment", "Handshake received: " + command);
            broadcastRobotStatus("Path Computed - Ready to Start Task");

            if (isAdded()) {
                // Use Toast to give immediate feedback to the user
                Toast.makeText(getContext(), "Computation Complete! Press Start.", Toast.LENGTH_LONG).show();
            }
        }

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

        lbm.registerReceiver(taskResetReceiver, new IntentFilter(Constants.INTENT_MESSAGE_SENT));
        lbm.registerReceiver(targetDetectedReceiver, new IntentFilter(Constants.INTENT_TARGET_DETECTED));
        lbm.registerReceiver(dirtyReceiver, new IntentFilter(Constants.INTENT_OBSTACLE_MAP_DIRTY));
        lbm.registerReceiver(endReceiver, new IntentFilter(Constants.INTENT_END_DETECTED));
        lbm.registerReceiver(syncRequestReceiver, new IntentFilter(Constants.INTENT_OBSTACLE_SYNC_REQUEST));

        // Listen for incoming msgs from RPi
        lbm.registerReceiver(incomingMessageReceiver, new IntentFilter(Constants.INTENT_MESSAGE_RECEIVED));
    }

    @Override
    public void onStop() {
        super.onStop();

        LocalBroadcastManager lbm = LocalBroadcastManager.getInstance(requireContext());
        lbm.unregisterReceiver(taskResetReceiver);
        lbm.unregisterReceiver(targetDetectedReceiver);
        lbm.unregisterReceiver(dirtyReceiver);
        lbm.unregisterReceiver(endReceiver);
        lbm.unregisterReceiver(incomingMessageReceiver);
        stopExploreTimer(); // safety
        lbm.unregisterReceiver(syncRequestReceiver);

        stopExploreTimer();
    }

    private void broadcastRobotStatus(String status) {
        if (!isAdded()) return;
        String finalStatus = isMapDirty ? status + " (Unsynced)" : status;
        Intent intent = new Intent(Constants.INTENT_ROBOT_ACTIVITY_STATUS);
        intent.putExtra("message", finalStatus);
        LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(intent);
    }

    private void startExploreTimer() {
        exploreStartMs = System.currentTimeMillis();
        exploreRunning = true;
        timerHandler.removeCallbacks(timerTick);
        timerHandler.post(timerTick);
    }

    private void stopExploreTimer() {
        if (!exploreRunning) return;
        exploreRunning = false;
        timerHandler.removeCallbacks(timerTick);

        long elapsed = System.currentTimeMillis() - exploreStartMs;
        broadcastTimer(elapsed, false);
    }

    private void broadcastTimer(long elapsedMs, boolean running) {
        if (!isAdded()) return;
        Intent i = new Intent(Constants.INTENT_TIMER_UPDATE);
        i.putExtra(Constants.EXTRA_TIMER_RUNNING, running);
        i.putExtra(Constants.EXTRA_TIMER_ELAPSED_MS, elapsedMs);
        LocalBroadcastManager.getInstance(requireContext()).sendBroadcast(i);
    }

}
