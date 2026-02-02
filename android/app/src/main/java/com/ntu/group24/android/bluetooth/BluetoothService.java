package com.ntu.group24.android.bluetooth;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
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

    private final BluetoothAdapter mBluetoothAdapter;
    private final Context mContext;

    private ConnectThread mConnectThread;
    private ConnectedThread mConnectedThread;

    private BluetoothDevice mDevice;          // last selected device
    private volatile boolean isConnected = false;

    public BluetoothService(Context context) {
        this.mContext = context;
        this.mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        sendStatusBroadcast("Idle");
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

    // Call this only if you want to explicitly close connection
    public synchronized void disconnect() {
        Log.d(TAG, "disconnect() called");

        if (mConnectThread != null) {
            mConnectThread.cancel();
            mConnectThread = null;
        }

        if (mConnectedThread != null) {
            mConnectedThread.cancel();
            mConnectedThread = null;
        }

        isConnected = false;
        sendStatusBroadcast("Disconnected");
    }

    /**
     * IMPORTANT: Do not use this to "reconnect".
     * This is now a safe "reset connect attempt" that will not kill a live connection.
     */
    public synchronized void start() {
        Log.d(TAG, "start() called (safe)");

        // only cancel connect attempts, do NOT cancel an active connection
        if (mConnectThread != null) {
            mConnectThread.cancel();
            mConnectThread = null;
        }
        // do not touch mConnectedThread here
    }

    public synchronized boolean isConnected() {
        return isConnected && mConnectedThread != null;
    }

    // Client Mode
    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice device;

        @SuppressLint("MissingPermission")
        public ConnectThread(BluetoothDevice device) {
            this.device = device;
            BluetoothSocket tmp = null;

            try {
                // Keep insecure if your RPi uses insecure RFCOMM
                tmp = device.createInsecureRfcommSocketToServiceRecord(Constants.MDP_UUID);
            } catch (IOException e) {
                Log.e(TAG, "ConnectThread create() failed", e);
            }

            mmSocket = tmp;
        }

        @SuppressLint("MissingPermission")
        public void run() {
            mBluetoothAdapter.cancelDiscovery();
            sendStatusBroadcast("Connecting...");

            if (mmSocket == null) {
                sendStatusBroadcast("Connection Failed");
                return;
            }

            try {
                Log.d(TAG, "Attempting connect to " + device.getName() + " " + device.getAddress());
                mmSocket.connect();
                Log.d(TAG, "Connect successful. isConnected=" + mmSocket.isConnected());

                synchronized (BluetoothService.this) {
                    mConnectThread = null;
                    manageConnectedSocket(mmSocket, device);
                }

            } catch (IOException e) {
                Log.e(TAG, "Connect failed", e);
                try { mmSocket.close(); } catch (IOException ignored) {}
                sendStatusBroadcast("Connection Failed");

                synchronized (BluetoothService.this) {
                    mConnectThread = null;
                    isConnected = false;
                }
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

    // Data Exchange
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
                Log.e(TAG, "ConnectedThread stream setup failed", e);
            }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;

            Log.d(TAG, "Streams ready. in=" + (mmInStream != null) + " out=" + (mmOutStream != null));
        }

        @Override
        public void run() {
            if (mmInStream == null || mmOutStream == null) {
                Log.e(TAG, "Streams null. Closing connection.");
                connectionLost("Streams null");
                return;
            }

            // Only broadcast connected once the thread is actually running with streams ready
            isConnected = true;
            sendStatusBroadcast("Connected");

            byte[] buffer = new byte[1024];
            StringBuilder sb = new StringBuilder();

            while (running) {
                try {
                    int bytes = mmInStream.read(buffer);

                    if (bytes == -1) {
                        connectionLost("Remote closed (-1)");
                        break;
                    }

                    String chunk = new String(buffer, 0, bytes, StandardCharsets.UTF_8);
                    sb.append(chunk);

                    int idx;
                    while ((idx = sb.indexOf("\n")) != -1) {
                        String line = sb.substring(0, idx).trim();
                        sb.delete(0, idx + 1);

                        if (!line.isEmpty()) {
                            sendMessageBroadcast(line);
                        }
                    }

                } catch (IOException e) {
                    connectionLost("Read error: " + e.getMessage());
                    break;
                }
            }
        }

        public void writeString(String msg) {
            if (msg == null) return;
            writeBytes((msg + "\n").getBytes(StandardCharsets.UTF_8));
        }

        public void writeBytes(byte[] bytes) {
            if (!running || mmOutStream == null || bytes == null) return;

            try {
                mmOutStream.write(bytes);
                mmOutStream.flush();
            } catch (IOException e) {
                connectionLost("Write error: " + e.getMessage());
            }
        }

        public void cancel() {
            running = false;
            try {
                if (mmSocket != null) mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "ConnectedThread cancel() failed", e);
            }
        }
    }

    public synchronized void connect(BluetoothDevice device) {
        mDevice = device;

        // Cancel any existing connect attempt
        if (mConnectThread != null) {
            mConnectThread.cancel();
            mConnectThread = null;
        }

        // Cancel existing active connection ONLY when user connects to a new device
        if (mConnectedThread != null) {
            mConnectedThread.cancel();
            mConnectedThread = null;
        }

        isConnected = false;

        mConnectThread = new ConnectThread(device);
        mConnectThread.start();
    }

    private synchronized void manageConnectedSocket(BluetoothSocket socket, BluetoothDevice device) {
        mDevice = device;

        // stop any in-progress connect attempt
        if (mConnectThread != null) {
            mConnectThread.cancel();
            mConnectThread = null;
        }

        // stop any existing active connection
        if (mConnectedThread != null) {
            mConnectedThread.cancel();
            mConnectedThread = null;
        }

        mConnectedThread = new ConnectedThread(socket);
        mConnectedThread.start();

        // IMPORTANT: Do NOT auto-send "HELLO" here.
        // Some RPi servers close if they receive unexpected data immediately.
    }

    private synchronized void connectionLost(String reason) {
        Log.d(TAG, "connectionLost(): " + reason);

        isConnected = false;

        if (mConnectedThread != null) {
            mConnectedThread.cancel();
            mConnectedThread = null;
        }

        sendStatusBroadcast("Disconnected");
    }

    public void write(String message) {
        ConnectedThread r;
        synchronized (this) {
            r = mConnectedThread;
        }

        if (r == null || !isConnected) {
            Log.e(TAG, "write() called but not connected");
            sendStatusBroadcast("Send Failed");
            return;
        }

        r.writeString(message);
    }

    // optional helper for UI reconnect
    public synchronized void reconnectLast() {
        if (mDevice != null) connect(mDevice);
        else sendStatusBroadcast("No last device");
    }
}
