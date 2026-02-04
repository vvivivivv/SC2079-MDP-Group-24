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
import java.nio.charset.Charset;

public class BluetoothService {
    private static final String TAG = "BluetoothService";
    private final BluetoothAdapter mBluetoothAdapter;
    private final Context mContext;

    private ConnectThread mConnectThread;
    private ConnectedThread mConnectedThread;
    private BluetoothDevice mDevice;

    public BluetoothService(Context context) {
        this.mContext = context;
        this.mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
    }

    private void sendStatusBroadcast(String status) {
        Intent intent = new Intent(Constants.INTENT_CONNECTION_STATUS);
        intent.putExtra("status", status);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);
    }

    private void sendMessageReceivedBroadcast(String message) {
        Intent intent = new Intent(Constants.INTENT_MESSAGE_RECEIVED);
        intent.putExtra("message", message);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);
    }

    // NEW: TX broadcast so Comms tab can show outgoing messages
    private void sendMessageSentBroadcast(String message) {
        Intent intent = new Intent(Constants.INTENT_MESSAGE_SENT);
        intent.putExtra("message", message);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);
    }

    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;

        @SuppressLint("MissingPermission")
        public ConnectThread(BluetoothDevice device) {
            mDevice = device;
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
            sendStatusBroadcast("Connecting...");

            try {
                Log.d(TAG, "Attempting connect to " + mDevice.getName() + " " + mDevice.getAddress());
                mmSocket.connect();
                Log.d(TAG, "Connect successful");
                manageConnectedSocket(mmSocket, mDevice);
            } catch (IOException e) {
                Log.e(TAG, "Connect failed", e);
                try { mmSocket.close(); } catch (IOException ignored) {}
                sendStatusBroadcast("Connection Failed");
            }
        }

        public void cancel() {
            try { mmSocket.close(); }
            catch (IOException e) { Log.e(TAG, "ConnectThread cancel() failed", e); }
        }
    }

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

            sendStatusBroadcast("Connected");
        }

        @Override
        public void run() {
            if (mmInStream == null) {
                sendStatusBroadcast("Disconnected");
                return;
            }

            byte[] buffer = new byte[1024];

            while (running) {
                try {
                    int bytes = mmInStream.read(buffer);
                    if (bytes == -1) break;

                    String incoming = new String(buffer, 0, bytes, Charset.defaultCharset());
                    // Optional: split lines if your RPi sends "\n"
                    sendMessageReceivedBroadcast(incoming.trim());

                } catch (IOException e) {
                    Log.d(TAG, "ConnectedThread lost connection", e);
                    break;
                }
            }

            sendStatusBroadcast("Disconnected");
            cancel();
        }

        public void writeRaw(byte[] bytes) throws IOException {
            if (mmOutStream == null) throw new IOException("OutputStream is null");
            mmOutStream.write(bytes);
            mmOutStream.flush();
        }

        public void cancel() {
            running = false;
            try { mmSocket.close(); }
            catch (IOException e) { Log.e(TAG, "ConnectedThread cancel() failed", e); }
        }
    }

    public void connect(BluetoothDevice device) {
        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        mConnectThread = new ConnectThread(device);
        mConnectThread.start();
    }

    private synchronized void manageConnectedSocket(BluetoothSocket socket, BluetoothDevice device) {
        mDevice = device;

        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        mConnectedThread = new ConnectedThread(socket);
        mConnectedThread.start();
    }

    // IMPORTANT: this is what your Map/Controls call.
    // Now it will show up in Comms as [TX] ...
    public void write(String message) {
        if (message == null) return;

        String msg = message.trim();
        if (msg.isEmpty()) return;

        // 1) Always log TX to Comms immediately
        sendMessageSentBroadcast(msg);

        // 2) Then attempt to send
        if (mConnectedThread == null) {
            sendStatusBroadcast("Send Failed");
            return;
        }

        try {
            // add newline if your RPi expects line-based messages
            mConnectedThread.writeRaw((msg + "\n").getBytes(Charset.defaultCharset()));
        } catch (IOException e) {
            Log.e(TAG, "Write failed", e);
            sendStatusBroadcast("Send Failed");
        }
    }

    private void sendTxBroadcast(String message) {
        Intent intent = new Intent(Constants.INTENT_MESSAGE_SENT);
        intent.putExtra("message", message);
        LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);
    }
}
