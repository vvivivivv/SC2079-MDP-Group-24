package com.ntu.group24.android.map;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.*;
import android.util.AttributeSet;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.ntu.group24.android.models.Obstacle;
import com.ntu.group24.android.models.Robot;
import com.ntu.group24.android.ui.MainActivity;
import com.ntu.group24.android.utils.Constants;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

public class GridMap extends View {
    private static final String TAG = "GridMap";
    private static final int GRID_SIZE = 20;
    private static final int START_CELLS = 4;
    private static final float INSET_PX = 24f;
    private static final float AXIS_MARGIN_PX = 56f;

    private final Robot robot = new Robot(1, 1, "N");

    // Pre-allocated objects for high-performance drawing (C.8)
    private final RectF tempRect = new RectF();
    private final RectF robotRect = new RectF();
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint borderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint obstaclePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint idTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint robotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint axisTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint startZonePaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private final Paint obstacleIdTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint faceLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint selectedPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private float cellSizePx;
    private float originX;
    private float originY;

    private final Map<Integer, Obstacle> obstacles = new LinkedHashMap<>();
    private @Nullable Cell selectedCell = null;
    private @Nullable Integer draggingId = null;
    private boolean isDragging = false;

    private GestureDetector gestureDetector;
    private OnObstacleTapListener obstacleTapListener;

    public interface OnObstacleTapListener {
        void onObstacleTapped(int obstacleId);
    }

    public void setOnObstacleTapListener(OnObstacleTapListener listener) {
        this.obstacleTapListener = listener;
    }

    public GridMap(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public interface OnCellTapListener {
        void onCellTapped(int x, int y);
    }
    private OnCellTapListener cellTapListener;

    public void setOnCellTapListener(OnCellTapListener listener) {
        this.cellTapListener = listener;
    }

    public void removeObstacle(int id) {
        obstacles.remove(id);
        invalidate();
    }

    public void setObstacleFace(int id, Obstacle.Dir face) {
        Obstacle o = obstacles.get(id);
        if (o != null) {
            o.setFace(face);
            invalidate();
        }
    }

    private void init() {
        gridPaint.setColor(Color.LTGRAY);
        gridPaint.setStrokeWidth(2f);
        gridPaint.setStyle(Paint.Style.STROKE);

        borderPaint.setColor(Color.BLACK);
        borderPaint.setStrokeWidth(5f);
        borderPaint.setStyle(Paint.Style.STROKE);

        obstaclePaint.setColor(Color.parseColor("#4A90E2"));
        obstaclePaint.setStyle(Paint.Style.FILL);

        selectedPaint.setColor(Color.parseColor("#443498DB"));
        selectedPaint.setStyle(Paint.Style.FILL);

        idTextPaint.setColor(Color.WHITE);
        idTextPaint.setTextAlign(Paint.Align.CENTER);
        idTextPaint.setTypeface(Typeface.DEFAULT_BOLD);
        idTextPaint.setTextSize(32f);

        axisTextPaint.setColor(Color.DKGRAY);
        axisTextPaint.setTextAlign(Paint.Align.CENTER);
        axisTextPaint.setTextSize(22f);

        startZonePaint.setColor(Color.parseColor("#334CAF50"));
        startZonePaint.setStyle(Paint.Style.FILL);

        robotPaint.setColor(Color.parseColor("#FFD700"));
        robotPaint.setStyle(Paint.Style.FILL);

        obstacleIdTextPaint.setColor(Color.WHITE);
        obstacleIdTextPaint.setTextAlign(Paint.Align.CENTER);
        obstacleIdTextPaint.setTypeface(Typeface.DEFAULT_BOLD);
        obstacleIdTextPaint.setTextSize(28f);

        faceLinePaint.setColor(Color.RED);
        faceLinePaint.setStrokeWidth(8f);
        faceLinePaint.setStyle(Paint.Style.STROKE);

        gestureDetector = new GestureDetector(getContext(), new GestureDetector.SimpleOnGestureListener() {
            @Override
            public boolean onDown(@NonNull MotionEvent e) { return true; }

            @Override
            public void onLongPress(@NonNull MotionEvent e) {
                Integer hit = findObstacleAt(e.getX(), e.getY());
                if (hit != null) {
                    draggingId = hit;
                    isDragging = true;
                    getParent().requestDisallowInterceptTouchEvent(true);
                }
            }

            @Override
            public boolean onSingleTapUp(@NonNull MotionEvent e) {
                Integer hit = findObstacleAt(e.getX(), e.getY());
                if (hit != null) {
                    if (obstacleTapListener != null) obstacleTapListener.onObstacleTapped(hit);
                    return true;
                }

                selectedCell = pointToCellModel(e.getX(), e.getY());
                invalidate();

                // Trigger listener > MapFragment opens "Add" dialog
                if (selectedCell != null && cellTapListener != null) {
                    cellTapListener.onCellTapped(selectedCell.x, selectedCell.y);
                }
                return true;
            }
        });
    }

    @Override
    protected void onDraw(@NonNull Canvas canvas) {
        super.onDraw(canvas);
        computeGeometry();
        canvas.drawColor(Color.WHITE);

        float left = originX;
        float top = originY;
        float right = originX + GRID_SIZE * cellSizePx;
        float bottom = originY + GRID_SIZE * cellSizePx;

        // 1) Start zone
        canvas.drawRect(startZoneRectModel(), startZonePaint);

        // 2) Selection Highlight
        if (selectedCell != null) {
            updateTempRect(selectedCell.x, selectedCell.y);
            canvas.drawRect(tempRect, selectedPaint);
        }

        // 3) Grid Lines
        for (int i = 0; i <= GRID_SIZE; i++) {
            canvas.drawLine(left + i * cellSizePx, top, left + i * cellSizePx, bottom, gridPaint);
            canvas.drawLine(left, top + i * cellSizePx, right, top + i * cellSizePx, gridPaint);
        }
        canvas.drawRect(left, top, right, bottom, borderPaint);

        // 4) Axes
        drawAxes(canvas, left, bottom);

        // 5) Draw Obstacles (C.5, C.9)
        for (Obstacle o : obstacles.values()) {
            updateTempRect(o.getX(), o.getY());
            canvas.drawRect(tempRect, obstaclePaint);

            String text = (o.getTargetId() != null) ? String.valueOf(o.getTargetId()) : String.valueOf(o.getId());
            Paint textPaint = (o.getTargetId() != null) ? idTextPaint : obstacleIdTextPaint;
            canvas.drawText(text, tempRect.centerX(), tempRect.centerY() + textPaint.getTextSize() / 3f, textPaint);

            drawFaceLine(canvas, tempRect, o.getFace());
        }

        // 6) Draw Robot (C.5, C.10)
        updateRobotRect(robot.getX(), robot.getY());
        canvas.drawRect(robotRect, robotPaint);
        drawRobotFront(canvas, robotRect, robot.getDirection());
    }

    private void drawFaceLine(Canvas canvas, RectF rect, Obstacle.Dir face) {
        float pad = 6f;
        switch (face) {
            case N: canvas.drawLine(rect.left + pad, rect.top + pad, rect.right - pad, rect.top + pad, faceLinePaint); break;
            case E: canvas.drawLine(rect.right - pad, rect.top + pad, rect.right - pad, rect.bottom - pad, faceLinePaint); break;
            case S: canvas.drawLine(rect.left + pad, rect.bottom - pad, rect.right - pad, rect.bottom - pad, faceLinePaint); break;
            case W: canvas.drawLine(rect.left + pad, rect.top + pad, rect.left + pad, rect.bottom - pad, faceLinePaint); break;
        }
    }

    private void drawRobotFront(Canvas canvas, RectF rect, String dir) {
        Paint frontPaint = new Paint();
        frontPaint.setColor(Color.BLACK);
        frontPaint.setStrokeWidth(12f);
        float pad = 5f;
        switch (dir) {
            case "N": canvas.drawLine(rect.left + pad, rect.top + pad, rect.right - pad, rect.top + pad, frontPaint); break;
            case "S": canvas.drawLine(rect.left + pad, rect.bottom - pad, rect.right - pad, rect.bottom - pad, frontPaint); break;
            case "E": canvas.drawLine(rect.right - pad, rect.top + pad, rect.right - pad, rect.bottom - pad, frontPaint); break;
            case "W": canvas.drawLine(rect.left + pad, rect.top + pad, rect.left + pad, rect.bottom - pad, frontPaint); break;
        }
    }

    private void drawAxes(Canvas canvas, float left, float bottom) {
        for (int x = 0; x < GRID_SIZE; x++) {
            canvas.drawText(String.valueOf(x), left + (x + 0.5f) * cellSizePx, bottom + 30f, axisTextPaint);
        }
        for (int y = 0; y < GRID_SIZE; y++) {
            int row = (GRID_SIZE - 1) - y;
            canvas.drawText(String.valueOf(y), left - 25f, originY + (row + 0.6f) * cellSizePx, axisTextPaint);
        }
    }

    @Override
    public boolean performClick() { return super.performClick(); }

    @SuppressLint("ClickableViewAccessibility")
    @Override
    public boolean onTouchEvent(MotionEvent event) {
        gestureDetector.onTouchEvent(event);
        if (event.getAction() == MotionEvent.ACTION_DOWN) performClick();

        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_MOVE:
                if (isDragging && draggingId != null) {
                    Cell c = pointToCellModel(event.getX(), event.getY());
                    if (c != null && !isCellOccupied(c.x, c.y, draggingId)) {
                        Obstacle o = obstacles.get(draggingId);
                        if (o != null) {
                            o.setX(c.x);
                            o.setY(c.y);
                            invalidate();
                        }
                    }
                }
                break;

            case MotionEvent.ACTION_UP:
                if (isDragging && draggingId != null) {
                    Obstacle o = obstacles.get(draggingId);
                    if (o != null) {
                        // Transmit ADD command when finger lifts (C.6)
                        String msg = String.format(Locale.US, Constants.OBSTACLE_ADD,
                                o.getId(), o.getX(), o.getY(), o.getFace().name());

                        MainActivity activity = (MainActivity) getContext();
                        if (activity.getBluetoothService() != null) {
                            activity.getBluetoothService().write(msg);
                        }
                    }
                    isDragging = false;
                    draggingId = null;
                    invalidate();
                }
                break;
        }
        return true;
    }

    private void computeGeometry() {
        cellSizePx = Math.min(getWidth() - 2*INSET_PX - AXIS_MARGIN_PX, getHeight() - 2*INSET_PX - AXIS_MARGIN_PX) / GRID_SIZE;
        originX = INSET_PX + AXIS_MARGIN_PX;
        originY = INSET_PX;
    }

    private @Nullable Cell pointToCellModel(float px, float py) {
        int x = (int) ((px - originX) / cellSizePx);
        int screenRow = (int) ((py - originY) / cellSizePx);
        if (x < 0 || x >= GRID_SIZE || screenRow < 0 || screenRow >= GRID_SIZE) return null;
        return new Cell(x, (GRID_SIZE - 1) - screenRow);
    }

    private @Nullable Integer findObstacleAt(float px, float py) {
        for (Obstacle o : obstacles.values()) {
            updateTempRect(o.getX(), o.getY());
            if (tempRect.contains(px, py)) return o.getId();
        }
        return null;
    }

    private boolean isCellOccupied(int x, int y, int currentDraggingId) {
        // 1. Check other obstacles
        for (Obstacle other : obstacles.values()) {
            if (other.getId() != currentDraggingId && other.getX() == x && other.getY() == y) {
                return true;
            }
        }

        // 2. Direct return for Robot footprint (3x3) or Start Zone (4x4)
        return (x >= robot.getX() && x <= robot.getX() + 2 &&
                y >= robot.getY() && y <= robot.getY() + 2) || (x < 4 && y < 4);
    }

    private void updateTempRect(int x, int y) {
        int screenRow = (GRID_SIZE - 1) - y;
        tempRect.set(originX + x * cellSizePx, originY + screenRow * cellSizePx,
                originX + (x + 1) * cellSizePx, originY + (screenRow + 1) * cellSizePx);
    }

    private void updateRobotRect(int x, int y) {
        int screenRowTop = (GRID_SIZE - 1) - (y + 2);
        robotRect.set(originX + x * cellSizePx, originY + screenRowTop * cellSizePx,
                originX + (x + 3) * cellSizePx, originY + (screenRowTop + 3) * cellSizePx);
    }

    private RectF startZoneRectModel() {
        updateTempRect(0, 0);
        float l = tempRect.left;
        float b = tempRect.bottom;
        updateTempRect(START_CELLS - 1, START_CELLS - 1);
        return new RectF(l, tempRect.top, tempRect.right, b);
    }

    public void applyCommand(String cmd) {
        if (cmd == null) return;
        String[] p = cmd.trim().split(",");
        if (p.length == 0) return;
        String op = p[0].trim().toUpperCase(Locale.US);

        switch (op) {
            case Constants.HEADER_ROBOT: // "ROBOT"
                if (p.length < 4) return;
                try {
                    robot.setX(Integer.parseInt(p[1]));
                    robot.setY(Integer.parseInt(p[2]));
                    robot.setDirection(p[3].toUpperCase(Locale.US));
                    invalidate(); // Refresh UI (C.10)
                } catch (Exception e) { Log.e(TAG, "Robot parse error"); }
                break;

            case Constants.HEADER_TARGET: // "TARGET"
                if (p.length < 3) return;
                try {
                    Obstacle o = obstacles.get(Integer.parseInt(p[1]));
                    if (o != null) {
                        o.setTargetId(Integer.parseInt(p[2]));
                        invalidate(); // Refresh UI (C.9)
                    }
                } catch (Exception e) { Log.e(TAG, "Target parse error"); }
                break;
        }
    }

    public void upsertObstacle(int id, int x, int y, Obstacle.Dir face) {
        obstacles.put(id, new Obstacle(id, x, y, face));
        invalidate();
    }

    private static class Cell {
        final int x, y;
        Cell(int x, int y) { this.x = x; this.y = y; }
    }
}