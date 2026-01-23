package com.ntu.group24.android.ui;

import android.os.Bundle;
import android.text.InputType;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.Toast;
import android.view.inputmethod.EditorInfo;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.fragment.app.Fragment;

import com.ntu.group24.android.R;
import com.ntu.group24.android.map.GridMap;
import com.ntu.group24.android.models.Obstacle;

public class MapFragment extends Fragment {

    private GridMap gridMap;

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {

        return inflater.inflate(R.layout.fragment_map_config, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view,
                              @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        gridMap = view.findViewById(R.id.gridMap);

        // Tap empty cell -> open "Add obstacle" dialog
        gridMap.setOnCellTapListener((x, y) -> {
            // x,y are internal 0-based model coords
            if (x <= 3 && y <= 3) {
                Toast.makeText(requireContext(), "Start zone reserved for robot", Toast.LENGTH_SHORT).show();
                return;
            }
            showAddObstacleDialog(x, y);
        });

        gridMap.setOnObstacleTapListener(obstacleId -> {
            showObstacleOptionsDialog(obstacleId);
        });
    }

    private void showAddObstacleDialog(int x0, int y0) {
        View dialogView = LayoutInflater.from(requireContext())
                .inflate(R.layout.dialog_add_obstacle, null);

        EditText idInput = dialogView.findViewById(R.id.inputObstacleId);
        Spinner faceSpinner = dialogView.findViewById(R.id.spinnerFace);

        // keyboard tuning (cannot fully remove mic/emoji, but helps)
        idInput.setInputType(InputType.TYPE_CLASS_NUMBER);
        idInput.setImeOptions(EditorInfo.IME_ACTION_DONE);

        ArrayAdapter<String> adapter = new ArrayAdapter<>(
                requireContext(),
                android.R.layout.simple_spinner_dropdown_item,
                new String[]{"N","E","S","W"}
        );
        faceSpinner.setAdapter(adapter);

        new AlertDialog.Builder(requireContext())
                .setTitle("Add Obstacle")
                .setMessage("Place at (" + (x0 + 1) + "," + (y0 + 1) + ")")
                .setView(dialogView)
                .setPositiveButton("Add", (d, which) -> {
                    String idStr = idInput.getText().toString().trim();
                    if (idStr.isEmpty()) {
                        Toast.makeText(requireContext(), "Enter an obstacle ID", Toast.LENGTH_SHORT).show();
                        return;
                    }

                    int id = Integer.parseInt(idStr);
                    String faceStr = (String) faceSpinner.getSelectedItem();
                    Obstacle.Dir face = parseDir(faceStr);

                    gridMap.upsertObstacle(id, x0, y0, face);

                    // later: send bluetooth command (choose your team format)
                    // String msg = "ADD," + id + "," + (x0 + 1) + "," + (y0 + 1) + "," + faceStr;
                })
                .setNegativeButton("Cancel", null)
                .show();
    }
    private void showObstacleOptionsDialog(int obstacleId) {
        String[] options = new String[]{"Change Face", "Delete"};

        new AlertDialog.Builder(requireContext())
                .setTitle("Obstacle " + obstacleId)
                .setItems(options, (dialog, which) -> {
                    if (which == 0) {
                        showChangeFaceDialog(obstacleId);
                    } else if (which == 1) {
                        gridMap.removeObstacle(obstacleId);
                        // later: bluetooth send SUB
                        // String msg = "SUB," + obstacleId;
                    }
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void showChangeFaceDialog(int obstacleId) {
        String[] faces = new String[]{"N", "E", "S", "W"};

        new AlertDialog.Builder(requireContext())
                .setTitle("Set Face for " + obstacleId)
                .setItems(faces, (dialog, which) -> {
                    Obstacle.Dir face = parseDir(faces[which]);
                    gridMap.setObstacleFace(obstacleId, face);

                    // later: bluetooth send FACE
                    // String msg = "FACE," + obstacleId + "," + faces[which];
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private Obstacle.Dir parseDir(String s) {
        switch (s) {
            case "N": return Obstacle.Dir.N;
            case "E": return Obstacle.Dir.E;
            case "S": return Obstacle.Dir.S;
            case "W": return Obstacle.Dir.W;
            default: return Obstacle.Dir.N;
        }
    }
}
