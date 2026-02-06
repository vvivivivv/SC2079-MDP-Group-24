package com.ntu.group24.android.bluetooth;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.ntu.group24.android.utils.Constants;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;

public class BluetoothService {
    private static final String TAG = "BluetoothService";
    private static final String APP_NAME = "MDP_Group24";
    private final BluetoothAdapter mBluetoothAdapter;
    private final Context mContext;

    private AcceptThread mAcceptThread;
    private ConnectThread mConnectThread;
    private ConnectedThread mConnectedThread;
    private BluetoothDevice mDevice;
    private volatile boolean isConnected = false;

    public BluetoothService(Context context) {
        this.mContext = context;
        this.mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        // Start listening for incoming connections immediately (for AMD Tool)
        start();
    }

    private void sendStatusBroadcast(String status) {
        Intent intent = new Intent(Constants.INTENT_CONNECTION_STATUS);
        intent.putExtra("status", status);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);
    }

    private void sendMessageBroadcast(String message) {
        Intent intent = new Intent(Constants.INTENT_MESSAGE_RECEIVED);
        intent.putExtra("message", message);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);
    }

    private void sendMessageSentBroadcast(String message) {
        Intent intent = new Intent(Constants.INTENT_MESSAGE_SENT);
        intent.putExtra("message", message);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);
    }

    // Thread 1: Server Mode (To receive connection from AMD Tool)
    private class AcceptThread extends Thread {
        private BluetoothServerSocket mmServerSocket;

        @SuppressLint("MissingPermission")
        public AcceptThread() {
            try {
                mmServerSocket = mBluetoothAdapter.listenUsingInsecureRfcommWithServiceRecord(APP_NAME, Constants.MDP_UUID);
            } catch (IOException e) {
                Log.e(TAG, "AcceptThread listen() failed", e);
            }
        }

        public void run() {
            BluetoothSocket socket;

            try {
                socket = mmServerSocket.accept();
            } catch (IOException e) {
                Log.e(TAG, "AcceptThread accept() failed", e);
                return;
            }

            if (socket != null) {
                synchronized (BluetoothService.this) {
                    manageConnectedSocket(socket, socket.getRemoteDevice());
                }
            }
        }

        public void cancel() {
            try {
                if (mmServerSocket != null) mmServerSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "AcceptThread close() failed", e);
            }
        }
    }

    // Thread 2: Client Mode (C.2) (To initiate connection to RPi)
    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice device;

        @SuppressLint("MissingPermission")
        public ConnectThread(BluetoothDevice device) {
            this.device = device;
            BluetoothSocket tmp = null;
            try {
                tmp = device.createInsecureRfcommSocketToServiceRecord(Constants.MDP_UUID);
            } catch (IOException e) {
                Log.e(TAG, "ConnectThread create() failed", e);
            }
            mmSocket = tmp;
        }

        @SuppressLint("MissingPermission")
        public void run() {
            mBluetoothAdapter.cancelDiscovery();

            if (mmSocket == null) {
                sendStatusBroadcast("Connection Failed");
                return;
            }

            try {
                mmSocket.connect();
                synchronized (BluetoothService.this) {
                    mConnectThread = null;
                    manageConnectedSocket(mmSocket, device);
                }
            } catch (IOException e) {
                Log.e(TAG, "Connect failed: " + e.getMessage());
                try { mmSocket.close(); } catch (IOException ignored) {}
                sendStatusBroadcast("Connection Failed");
            }
        }

        public void cancel() {
            try {
                if (mmSocket != null) mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "ConnectThread cancel() failed", e);
            }
        }
    }

    // Thread 3: Data Exchange (C.1)
    private class ConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;
        private volatile boolean running = true;

        public ConnectedThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Log.e(TAG, "Stream setup failed", e);
            }
            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        @Override
        public void run() {
            if (mmInStream == null || mmOutStream == null) {
                connectionLost();
                return;
            }

            isConnected = true;
            sendStatusBroadcast("Connected");

            byte[] buffer = new byte[1024];
            StringBuilder sb = new StringBuilder();

            while (running) {
                try {
                    int bytes = mmInStream.read(buffer);
                    if (bytes == -1) {
                        Log.d(TAG, "Remote closed stream (-1)");
                        connectionLost();
                        break;
                    }

                    String chunk = new String(buffer, 0, bytes, StandardCharsets.UTF_8);
                    Log.d(TAG, "RX raw chunk: " + chunk);

                    sb.append(chunk);

                    boolean emittedLine = false;

                    // 1) Emit newline-delimited lines (handles \n and \r\n)
                    int idx;
                    while ((idx = sb.indexOf("\n")) != -1) {
                        String line = sb.substring(0, idx).replace("\r", "").trim();
                        sb.delete(0, idx + 1);

                        if (!line.isEmpty()) {
                            sendMessageBroadcast(line);
                            emittedLine = true;
                        }
                    }


                    // 2) If AMD tool sends WITHOUT newline, emit what we got
                    if (!emittedLine) {
                        String msg = sb.toString().replace("\r", "").trim();
                        if (!msg.isEmpty()) {
                            sendMessageBroadcast(msg);
                            sb.setLength(0);
                        }
                    }

                } catch (IOException e) {
                    Log.e(TAG, "Read failed, connection lost", e);
                    connectionLost();
                    isConnected = false;
                    sendStatusBroadcast("Disconnected");
                    break;
                }
            }
        }

        public void write(byte[] bytes) {
            try {
                mmOutStream.write(bytes);
                mmOutStream.flush();
            } catch (IOException e) {
                Log.e(TAG, "Write failed", e);
                isConnected = false;
                sendStatusBroadcast("Disconnected");
            }
        }

        public void cancel() {
            running = false;
            try { mmSocket.close(); } catch (IOException e) { Log.e(TAG, "Close failed", e); }
        }
    }

    public synchronized void start() {
        Log.d(TAG, "start(): server listen only");

        if (mAcceptThread == null) {
            mAcceptThread = new AcceptThread();
            mAcceptThread.start();
        }
        sendStatusBroadcast("Idle/Listening");
    }

    public synchronized void connect(BluetoothDevice device) {
        // When initiating a connection, stop listening as server temporarily
        if (mAcceptThread != null) { mAcceptThread.cancel(); mAcceptThread = null; }
        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        mConnectThread = new ConnectThread(device);
        mConnectThread.start();
        sendStatusBroadcast("Connecting...");
    }

    private synchronized void manageConnectedSocket(BluetoothSocket socket, BluetoothDevice device) {
        mDevice = device;

        String addr = "unknown";
        try { addr = device.getAddress(); } catch (Exception ignored) {}

        String name = "unknown";
        if (androidx.core.content.ContextCompat.checkSelfPermission(
                mContext, android.Manifest.permission.BLUETOOTH_CONNECT
        ) == android.content.pm.PackageManager.PERMISSION_GRANTED) {
            try { name = device.getName(); } catch (Exception ignored) {}
        }

        Log.d(TAG, "manageConnectedSocket(): CONNECTED TO = " + name + " (" + addr + ")");

        if (mAcceptThread != null) { mAcceptThread.cancel(); mAcceptThread = null; }
        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        mConnectedThread = new ConnectedThread(socket);
        mConnectedThread.start();
    }



    @SuppressLint("MissingPermission")
    public String getConnectedDeviceName() {
        if (isConnected && mDevice != null) {
            return mDevice.getName();
        }
        return "None";
    }

    private synchronized void connectionLost() {
        isConnected = false;
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }
        sendStatusBroadcast("Disconnected");
        // Re-listen so RPi can reconnect (C.8)
        start();
    }

    public void write(String message) {
        if (message == null) return;

        message = message.trim();

        // 1) Always show in COMMS as TX
        Intent tx = new Intent(Constants.INTENT_MESSAGE_SENT);
        tx.putExtra("message", message);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(tx);

        ConnectedThread r;
        synchronized (this) {
            r = mConnectedThread;
        }

        if (!isConnected || r == null) {
            Log.e(TAG, "Not connected, cannot write: " + message);
            sendStatusBroadcast("Send Failed");
            return;
        }

        // 2) Movement commands: DO NOT append newline
        boolean isMove =
                message.equals(Constants.MOVE_FORWARD) ||
                        message.equals(Constants.MOVE_BACKWARD) ||
                        message.equals(Constants.TURN_LEFT) ||
                        message.equals(Constants.TURN_RIGHT);

        byte[] payload;
        if (isMove) {
            payload = message.getBytes(StandardCharsets.UTF_8); // "f", "b", "tl", "tr"
        } else {
            payload = (message + "\n").getBytes(StandardCharsets.UTF_8); // other commands/messages
        }

        r.write(payload);
        Log.d(TAG, "Sent bytes: '" + message + "'" + (isMove ? " (no newline)" : " (with newline)"));
    }

}