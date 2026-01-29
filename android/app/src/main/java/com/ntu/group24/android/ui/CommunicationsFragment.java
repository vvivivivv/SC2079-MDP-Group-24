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
import android.text.SpannableString;
import android.text.Spanned;
import android.text.style.ForegroundColorSpan;
import android.graphics.Color;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.ntu.group24.android.R;
import com.ntu.group24.android.utils.Constants;

public class CommunicationsFragment extends Fragment {

    private TextView tvLog;
    private TextView tvStatus;

    private final BroadcastReceiver commsReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!isAdded() || getContext() == null) return;

            String action = intent.getAction();
            if (action == null) return;

            if (Constants.INTENT_CONNECTION_STATUS.equals(action)) {
                String status = intent.getStringExtra("status");
                if (status != null && tvStatus != null) {
                    tvStatus.setText("Status: " + status);

                    if (status.equalsIgnoreCase("Connected")) {
                        tvStatus.setBackgroundColor(Color.parseColor("#C8E6C9")); // light green
                    } else if (status.equalsIgnoreCase("Disconnected")) {
                        tvStatus.setBackgroundColor(Color.parseColor("#FFCDD2")); // light red
                    } else {
                        tvStatus.setBackgroundColor(Color.parseColor("#EEEEEE")); // neutral
                    }

                    appendLine("[STATUS] " + status);
                }
                return;
            }


            if (Constants.INTENT_MESSAGE_RECEIVED.equals(action)) {
                String msg = intent.getStringExtra("message");
                if (msg == null) return;

                // C.4: do not spam coordinates in the comms log
                if (msg.startsWith(Constants.HEADER_ROBOT) || msg.startsWith(Constants.HEADER_TARGET)) return;

                appendLine("[RX] " + msg.trim());
                return;
            }

            if (Constants.INTENT_MESSAGE_SENT.equals(action)) {
                String cmd = intent.getStringExtra("message");
                if (cmd == null) return;
                appendLine("[TX] " + cmd.trim());
            }
        }
    };

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             @Nullable ViewGroup container,
                             @Nullable Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.fragment_communications, container, false);
        tvLog = root.findViewById(R.id.tvCommsLog);
        tvStatus = root.findViewById(R.id.tvCommsStatus);

        tvLog.setMovementMethod(new ScrollingMovementMethod());
        return root;
    }

    @Override
    public void onStart() {
        super.onStart();
        IntentFilter filter = new IntentFilter();
        filter.addAction(Constants.INTENT_MESSAGE_RECEIVED);
        filter.addAction(Constants.INTENT_CONNECTION_STATUS);
        filter.addAction(Constants.INTENT_MESSAGE_SENT);
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
        if (scrollAmount > 0) tvLog.scrollTo(0, scrollAmount);
        else tvLog.scrollTo(0, 0);
    }

}
