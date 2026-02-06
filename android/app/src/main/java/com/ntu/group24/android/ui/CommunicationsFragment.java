package com.ntu.group24.android.ui;
// C.4
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.Button;
import android.text.SpannableString;
import android.text.Spanned;
import android.text.style.ForegroundColorSpan;
import android.graphics.Color;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.ntu.group24.android.R;
import com.ntu.group24.android.utils.Constants;

public class CommunicationsFragment extends Fragment {

    private TextView tvLog;
    private TextView tvStatus;
    private android.widget.EditText etInput;

    private final BroadcastReceiver commsReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded() || getContext() == null) return;

            String action = intent.getAction();
            if (action == null) return;

            switch (action) {
                case Constants.INTENT_CONNECTION_STATUS:
                    handleStatusChange(intent);
                    break;
                case Constants.INTENT_MESSAGE_RECEIVED:
                    handleMessageReceived(intent);
                    break;
                case Constants.INTENT_MESSAGE_SENT:
                    handleMessageSent(intent);
                    break;
                case Constants.INTENT_ROBOT_ACTIVITY_STATUS:
                    String activityStatus = intent.getStringExtra("message");
                    if (activityStatus != null && tvStatus != null) {
                        tvStatus.setText(activityStatus);
                        tvStatus.setBackgroundColor(Color.parseColor("#BBDEFB"));
                        appendLine("[RX] " + activityStatus);
                    }
                    break;
            }
        }
    };

    private void handleStatusChange(Intent intent) {
        String status = intent.getStringExtra("status");
        if (status != null && tvStatus != null) {
            tvStatus.setText(getString(R.string.status_format, status));

            int backgroundColor;
            switch (status.toLowerCase()) {
                case "connected":
                    backgroundColor = Color.parseColor("#C8E6C9"); // light green
                    break;
                case "disconnected":
                    backgroundColor = Color.parseColor("#FFCDD2"); // light red
                    break;
                default:
                    backgroundColor = Color.parseColor("#EEEEEE"); // neutral
                    break;
            }
            tvStatus.setBackgroundColor(backgroundColor);
            appendLine("[STATUS] " + status);
        }
    }

    private void handleMessageReceived(Intent intent) {
        String msg = intent.getStringExtra("message");
        if (msg == null) return;

        Log.d("CommsFragment", "Fragment received RX: " + msg);

        appendLine("[RX] " + msg.trim());
    }

    private void handleMessageSent(Intent intent) {
        String cmd = intent.getStringExtra("message");
        if (cmd == null) return;
        appendLine("[TX] " + cmd.trim());
    }

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.fragment_communications, container, false);
        tvLog = root.findViewById(R.id.tvCommsLog);
        tvStatus = root.findViewById(R.id.tvCommsStatus);

        tvLog.setMovementMethod(new ScrollingMovementMethod());

        // Send message to AMD tool (C.1)
        etInput = root.findViewById(R.id.etInput);
        Button btnSend = root.findViewById(R.id.btnSend);

        btnSend.setOnClickListener(v -> {
            String message = etInput.getText().toString().trim();
            if (!message.isEmpty()) {
                MainActivity activity = (MainActivity) getActivity();
                if (activity != null && activity.getBluetoothService() != null) {
                    activity.getBluetoothService().write(message);

                    etInput.setText("");
                } else {
                    Toast.makeText(requireContext(), R.string.msg_not_connected, Toast.LENGTH_SHORT).show();
                }
            }
        });

        return root;
    }

    @Override
    public void onStart() {
        super.onStart();
        IntentFilter filter = new IntentFilter();
        filter.addAction(Constants.INTENT_MESSAGE_RECEIVED);
        filter.addAction(Constants.INTENT_CONNECTION_STATUS);
        filter.addAction(Constants.INTENT_MESSAGE_SENT);
        filter.addAction(Constants.INTENT_ROBOT_ACTIVITY_STATUS);
        LocalBroadcastManager.getInstance(requireContext()).registerReceiver(commsReceiver, filter);
    }

    @Override
    public void onStop() {
        super.onStop();
        LocalBroadcastManager.getInstance(requireContext()).unregisterReceiver(commsReceiver);
    }

    private void appendLine(String line) {
        if (tvLog == null) return;

        int color = Color.DKGRAY;

        if (line.startsWith("[TX]")) color = Color.parseColor("#1565C0");      // blue
        else if (line.startsWith("[RX]")) color = Color.parseColor("#2E7D32"); // green
        else if (line.startsWith("[STATUS]")) color = Color.parseColor("#616161"); // gray

        SpannableString ss = new SpannableString(line + "\n");
        ss.setSpan(new ForegroundColorSpan(color), 0, ss.length(), Spanned.SPAN_EXCLUSIVE_EXCLUSIVE);
        tvLog.append(ss);

        int scrollAmount = tvLog.getLayout() == null ? 0
                : tvLog.getLayout().getLineTop(tvLog.getLineCount()) - tvLog.getHeight();

        tvLog.scrollTo(0, Math.max(0, scrollAmount));
    }

}
