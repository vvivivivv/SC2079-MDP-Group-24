package com.ntu.group24.android.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class CommsLog {
    private static final int MAX_LINES = 300;
    private static final ArrayList<String> LINES = new ArrayList<>();

    private CommsLog() {}

    public static synchronized void add(String line) {
        if (line == null) return;
        LINES.add(line);
        if (LINES.size() > MAX_LINES) {
            LINES.subList(0, LINES.size() - MAX_LINES).clear();
        }
    }

    public static synchronized List<String> snapshot() {
        return new ArrayList<>(LINES);
    }

    public static synchronized void clear() {
        LINES.clear();
    }
}
