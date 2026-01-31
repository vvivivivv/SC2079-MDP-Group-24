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
import java.lang.reflect.Method;
import java.nio.charset.Charset;

public class BluetoothService {
    private static final String TAG = "BluetoothService";
    private static final String APP_NAME = "MDP_Group24";
    private final BluetoothAdapter mBluetoothAdapter;
    private final Context mContext;

    private AcceptThread mAcceptThread;
    private ConnectThread mConnectThread;
    private ConnectedThread mConnectedThread;
    private BluetoothDevice mDevice;

    public BluetoothService(Context context) {
        this.mContext = context;
        this.mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        //if (mBluetoothAdapter != null && mBluetoothAdapter.isEnabled()) {
         //   start();
        //} else {
         //   Log.e(TAG, "Bluetooth Adapter not available or disabled");
        //}
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

    // Thread 1: Server Mode
    private class AcceptThread extends Thread {
        private BluetoothServerSocket mmServerSocket;

        @SuppressLint("MissingPermission")
        public AcceptThread() {
            try {
                mmServerSocket = mBluetoothAdapter.listenUsingInsecureRfcommWithServiceRecord(APP_NAME, Constants.MDP_UUID);
            } catch (IOException | SecurityException e) {
                Log.e(TAG, "AcceptThread listen() failed", e);
            }
        }

        public void run() {
            BluetoothSocket socket = null;
            try {
                if (mmServerSocket != null) {
                    socket = mmServerSocket.accept();
                }
            } catch (IOException e) {
                Log.e(TAG, "AcceptThread accept() failed", e);
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

    // Thread 2: Client Mode (C.2)
    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;

        @SuppressLint("MissingPermission")
        public ConnectThread(BluetoothDevice device) {
            mDevice = device;
            BluetoothSocket tmp = null;
            try {
                // Method 1: Standard
                tmp = device.createInsecureRfcommSocketToServiceRecord(Constants.MDP_UUID);
            } catch (IOException e) {
                Log.e(TAG, "Standard socket failed, trying Reflection...");
                try {
                    // Method 2: Reflection
                    Method m = device.getClass().getMethod("createRfcommSocket", int.class);
                    tmp = (BluetoothSocket) m.invoke(device, 1);
                } catch (Exception e2) {
                    Log.e(TAG, "Reflection socket failed", e2);
                }
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
                manageConnectedSocket(mmSocket, mDevice);
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

        public ConnectedThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            sendStatusBroadcast("Connected");

            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) {
                Log.e(TAG, "Stream setup failed", e);
            }
            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        public void run() {
            byte[] buffer = new byte[1024];
            int bytes;
            while (true) {
                try {
                    if (mmInStream != null) {
                        bytes = mmInStream.read(buffer);
                        String incomingMessage = new String(buffer, 0, bytes);
                        sendMessageBroadcast(incomingMessage);
                    }
                } catch (IOException e) {
                    Log.e(TAG, "ConnectedThread lost connection", e);
                    sendStatusBroadcast("Disconnected");
                    // Re-listen automatically so RPi can reconnect (C.8)
                    BluetoothService.this.start();
                    break;
                }
            }
        }

        public void write(byte[] bytes) {
            try {
                if (mmOutStream != null) {mmOutStream.write(bytes);}
            } catch (IOException e) {
                Log.e(TAG, "Write failed", e);
                sendStatusBroadcast("Send Failed");
            }
        }

        public void cancel() {
            try {
                if (mmSocket != null) mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "ConnectedThread close failed", e);
            }
        }
    }

    public synchronized void start() {
        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        if (mAcceptThread == null) {
            mAcceptThread = new AcceptThread();
            mAcceptThread.start();
        }
    }

    public synchronized void connect(BluetoothDevice device) {
        if (mAcceptThread != null) { mAcceptThread.cancel(); mAcceptThread = null; }
        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }
        if (mConnectedThread != null) { mConnectedThread.cancel(); mConnectedThread = null; }

        mConnectThread = new ConnectThread(device);
        mConnectThread.start();
    }

    private synchronized void manageConnectedSocket(BluetoothSocket socket, BluetoothDevice device) {
        mDevice = device;
        if (mAcceptThread != null) { mAcceptThread.cancel(); mAcceptThread = null; }
        if (mConnectThread != null) { mConnectThread.cancel(); mConnectThread = null; }

        mConnectedThread = new ConnectedThread(socket);
        mConnectedThread.start();
    }

    public void write(String message) {
        if (mConnectedThread != null) {
            // Send the physical bytes to the AMD Tool
            mConnectedThread.write(message.getBytes(Charset.defaultCharset()));

            Intent intent = new Intent(Constants.INTENT_MESSAGE_SENT);
            intent.putExtra("message", message);
            LocalBroadcastManager.getInstance(mContext).sendBroadcast(intent);

            Log.d(TAG, "Sent message: " + message);
        } else {
            Log.e(TAG, "Not connected, cannot write");
        }
    }
}