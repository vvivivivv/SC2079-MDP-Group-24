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

        // Observe changes from control fragment
        robotViewModel.getMoveRequest().observe(getViewLifecycleOwner(), direction -> {
            if (direction != null && gridMap != null) {
                gridMap.moveRobotManually(direction);
                robotViewModel.requestMovement(null);
            }
        });

        robotViewModel.getIncomingCommand().observe(getViewLifecycleOwner(), command -> {
            if (command != null && gridMap != null) {
                gridMap.applyCommand(command);
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

    public void handleIncomingCommand(String command) {
        if (gridMap != null) {
            gridMap.applyCommand(command);
        }
    }
}