package com.ntu.group24.android.utils;

import java.util.UUID;

public class Constants {
    // Standard SerialPortService ID for Bluetooth Classic (RFCOMM)
    public static final UUID MDP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    // Outgoing commands from Android to RPi (C.3, C.6, C.7)
    // Movement (C.3)
    public static final String MOVE_FORWARD = "f";
    public static final String MOVE_BACKWARD = "b";
    public static final String TURN_LEFT = "tl";
    public static final String TURN_RIGHT = "tr";

    // Task 1
    public static final String START_EXPLORATION = "START_EXPLORE";
    // Task 2
    public static final String START_FASTEST_PATH = "START_FASTEST";

    // Obstacle management (C.6, C.7)
    // Use String.format(OBSTACLE_ADD, id, x, y, face)
    // Expected: "ADD,1,5,10,N"
    public static final String OBSTACLE_ADD = "ADD,%d,%d,%d,%s";
    public static final String OBSTACLE_REMOVE = "SUB,%d";

    // Incoming data headers from RPi to Android (C.9, C.10)
    // Position update
    public static final String HEADER_ROBOT = "ROBOT";
    // Image ID
    public static final String HEADER_TARGET = "TARGET";
    // Status message (C.4)
    public static final String HEADER_STATUS = "STATUS";

    // Shared preference keys for saving map data
    public static final String PREF_MAP_CONFIG = "map_config";
    public static final String PREF_LAST_CONNECTED_DEVICE = "last_device";

    // Broadcast names
    public static final String INTENT_MESSAGE_RECEIVED = "com.ntu.mdp.MESSAGE_RECEIVED";
    public static final String INTENT_CONNECTION_STATUS = "com.ntu.mdp.CONNECTION_STATUS";
}