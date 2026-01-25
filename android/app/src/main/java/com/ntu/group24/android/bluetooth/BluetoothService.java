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

    // Thread 1: Server Mode
    private class AcceptThread extends Thread {
        private final BluetoothServerSocket mmServerSocket;

        @SuppressLint("MissingPermission")
        public AcceptThread() {
            BluetoothServerSocket tmp = null;
            try {
                tmp = mBluetoothAdapter.listenUsingInsecureRfcommWithServiceRecord(APP_NAME, Constants.MDP_UUID);
            } catch (IOException e) {
                Log.e(TAG, "AcceptThread listen() failed", e);
            }
            mmServerSocket = tmp;
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
                tmp = device.createRfcommSocketToServiceRecord(Constants.MDP_UUID);
            } catch (IOException e) {
                Log.e(TAG, "ConnectThread create() failed", e);
            }
            mmSocket = tmp;
        }

        @SuppressLint("MissingPermission")
        public void run() {
            mBluetoothAdapter.cancelDiscovery();
            try {
                mmSocket.connect();
                manageConnectedSocket(mmSocket, mDevice);
            } catch (IOException e) {
                try {
                    mmSocket.close();
                } catch (IOException e1) {
                    Log.e(TAG, "ConnectThread close failed during failure", e1);
                }
                sendStatusBroadcast("Connection Failed");
            }
        }

        public void cancel() {
            try {
                mmSocket.close();
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
                Log.e(TAG, "ConnectedThread stream setup failed", e);
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
                    Log.d(TAG, "ConnectedThread lost connection");
                    sendStatusBroadcast("Disconnected");
                    // Re-listen automatically so RPi can reconnect (C.8)
                    BluetoothService.this.start();
                    break;
                }
            }
        }

        public void write(byte[] bytes) {
            try {
                if (mmOutStream != null) mmOutStream.write(bytes);
            } catch (IOException e) {
                Log.e(TAG, "Write failed", e);
                sendStatusBroadcast("Send Failed");
            }
        }

        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "ConnectedThread cancel() failed", e);
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

    public void connect(BluetoothDevice device) {
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
            mConnectedThread.write(message.getBytes(Charset.defaultCharset()));
        }
    }
}