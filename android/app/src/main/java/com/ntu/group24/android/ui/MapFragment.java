package com.ntu.group24.android.ui;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.fragment.app.Fragment;
import com.ntu.group24.android.R;
import com.ntu.group24.android.map.GridMap;
import com.ntu.group24.android.models.Obstacle;
import com.ntu.group24.android.utils.Constants;
import java.util.Locale;

public class MapFragment extends Fragment {

    private GridMap gridMap;

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        // Fragment uses map_config layout which contains the GridMap
        return inflater.inflate(R.layout.fragment_map_config, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        gridMap = view.findViewById(R.id.gridMap);

        if (gridMap == null) {
            return;
        }

        // Tap empty cell to add obstacle (C.6)
        gridMap.setOnCellTapListener((x, y) -> {
            boolean inStartZone = (x >= 0 && x <= 3) && (y >= 0 && y <= 3);
            if (inStartZone) {
                Toast.makeText(requireContext(), "Start zone reserved", Toast.LENGTH_SHORT).show();
            } else {
                showAddObstacleDialog(x, y);
            }
        });
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
                    MainActivity activity = (MainActivity) requireActivity();
                    if (activity.getBluetoothService() != null) {
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