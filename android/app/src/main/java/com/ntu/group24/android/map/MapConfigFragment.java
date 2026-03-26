package com.ntu.group24.android.map;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import com.ntu.group24.android.R;

public class MapConfigFragment extends Fragment {

    private GridMap gridMap;

    public MapConfigFragment() {
        // Required empty public constructor
    }

    @Nullable
    @Override
    public View onCreateView(
            @NonNull LayoutInflater inflater,
            @Nullable ViewGroup container,
            @Nullable Bundle savedInstanceState
    ) {
        // 1) Inflate the fragment layout
        View view = inflater.inflate(R.layout.fragment_map_config, container, false);

        // 2) Get reference to GridMap
        gridMap = view.findViewById(R.id.gridMap);

        // 3) Temporary test commands (remove later)
        gridMap.applyCommand("ADD,1,5,10,N");
        gridMap.applyCommand("ADD,2,7,12,E");
        gridMap.applyCommand("TARGET,2,11");

        return view;
    }

    // Optional: expose this so Bluetooth / ProtocolHandler can call it
    public void handleIncomingCommand(String command) {
        if (gridMap != null) {
            gridMap.applyCommand(command);
        }
    }
}
