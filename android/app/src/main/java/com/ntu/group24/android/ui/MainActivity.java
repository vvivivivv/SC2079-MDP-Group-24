package com.ntu.group24.android.ui;

import android.Manifest;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import androidx.viewpager2.widget.ViewPager2;
import com.google.android.material.tabs.TabLayout;
import com.google.android.material.tabs.TabLayoutMediator;
import com.ntu.group24.android.R;
import com.ntu.group24.android.bluetooth.BluetoothService;
import com.ntu.group24.android.utils.Constants;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "MainActivity";
    private static final int REQUEST_CODE_PERMISSIONS = 101;
    private BluetoothService mBluetoothService;

    private final BroadcastReceiver mMessageReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (Constants.INTENT_MESSAGE_RECEIVED.equals(intent.getAction())) {
                String message = intent.getStringExtra("message");

                if (message != null && (message.startsWith(Constants.HEADER_ROBOT) || message.startsWith(Constants.HEADER_TARGET))) {
                    Fragment fragment = getSupportFragmentManager().findFragmentByTag("f0");
                    if (fragment instanceof MapFragment) {
                        ((MapFragment) fragment).handleIncomingCommand(message);
                    }
                }
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // 1. Request permissions immediately (Android 14 requirement)
        checkPermissions();

        // 2. Initialise bluetooth service (C.8)
        try {
            mBluetoothService = new BluetoothService(this);
        } catch (Exception e) {
            Log.e(TAG, "BT Service failed to start: " + e.getMessage());
        }

        TabLayout tabLayout = findViewById(R.id.tab_layout);
        ViewPager2 viewPager = findViewById(R.id.view_pager);
        SectionsPagerAdapter adapter = new SectionsPagerAdapter(this);
        viewPager.setAdapter(adapter);

        // Connect TabLayout and ViewPager2 (Added COMMS tab)
        new TabLayoutMediator(tabLayout, viewPager, (tab, position) -> {
            switch (position) {
                case 0: tab.setText("MAP"); break;
                case 1: tab.setText("CONTROLS"); break;
                case 2: tab.setText("COMMS"); break;
                case 3: tab.setText("BLUETOOTH"); break;
            }
        }).attach();
    }

    @Override
    protected void onResume() {
        super.onResume();
        IntentFilter filter = new IntentFilter(Constants.INTENT_MESSAGE_RECEIVED);
        LocalBroadcastManager.getInstance(this).registerReceiver(mMessageReceiver, filter);
    }

    @Override
    protected void onPause() {
        super.onPause();
        LocalBroadcastManager.getInstance(this).unregisterReceiver(mMessageReceiver);
    }

    public BluetoothService getBluetoothService() {
        return mBluetoothService;
    }

    private void checkPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ActivityCompat.requestPermissions(this,
                    new String[]{
                            Manifest.permission.BLUETOOTH_SCAN,
                            Manifest.permission.BLUETOOTH_CONNECT,
                            Manifest.permission.BLUETOOTH_ADVERTISE,
                            Manifest.permission.ACCESS_FINE_LOCATION,
                            Manifest.permission.ACCESS_COARSE_LOCATION
                    }, REQUEST_CODE_PERMISSIONS);
        } else {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                    REQUEST_CODE_PERMISSIONS);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_CODE_PERMISSIONS) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                Toast.makeText(this, "Permissions Granted", Toast.LENGTH_SHORT).show();
            } else {
                // Graceful failure if permissions are denied (C.8)
                Toast.makeText(this, "Permissions Denied! Bluetooth will not work.", Toast.LENGTH_LONG).show();
            }
        }
    }
}