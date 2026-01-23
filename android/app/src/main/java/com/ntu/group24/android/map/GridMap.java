package com.ntu.group24.android.map;

import android.content.Context;
import android.graphics.*;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.view.GestureDetector;
import android.view.ViewConfiguration;

import androidx.annotation.Nullable;

import com.ntu.group24.android.models.Obstacle;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

public class GridMap extends View {

    private static final int GRID_SIZE = 20;

    // Drawing helpers
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint borderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint obstaclePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint selectedPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint idTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint faceTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint targetTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Axis + start zone paints
    private final Paint axisTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint startZonePaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Layout spacing
    private float insetPx;      // outer padding from screen edge
    private float axisMarginPx; // space reserved for axis labels

    // Start area: 40cm x 40cm = 4x4 cells
    private static final int START_CELLS = 4;

    // Optional listener for taps
    public interface OnCellTapListener {
        void onCellTapped(int x, int y);
    }

    private OnCellTapListener cellTapListener;

    // State
    private final Map<Integer, Obstacle> obstacles = new LinkedHashMap<>();
    private @Nullable Cell selectedCell = null;

    // Drag state
    private @Nullable Integer draggingId = null;

    // Geometry
    private float cellSizePx;
    private float originX;
    private float originY;

    public GridMap(Context context) {
        super(context);
        init();
    }

    public GridMap(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public GridMap(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }
    private final Paint obstacleIdTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint faceLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private GestureDetector gestureDetector;
    private boolean isDragging = false;

    // Tap obstacle listener so Fragment can show delete/face dialog
    public interface OnObstacleTapListener {
        void onObstacleTapped(int obstacleId);
    }
    private OnObstacleTapListener obstacleTapListener;

    public void setOnObstacleTapListener(OnObstacleTapListener listener) {
        this.obstacleTapListener = listener;
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

        selectedPaint.setColor(Color.parseColor("#553498DB"));
        selectedPaint.setStyle(Paint.Style.FILL);

        idTextPaint.setColor(Color.WHITE);
        idTextPaint.setTextAlign(Paint.Align.CENTER);
        idTextPaint.setTypeface(Typeface.DEFAULT_BOLD);
        idTextPaint.setTextSize(32f);

        faceTextPaint.setColor(Color.WHITE);
        faceTextPaint.setTextAlign(Paint.Align.CENTER);
        faceTextPaint.setTypeface(Typeface.MONOSPACE);
        faceTextPaint.setTextSize(24f);

        targetTextPaint.setColor(Color.WHITE);
        targetTextPaint.setTextAlign(Paint.Align.CENTER);
        targetTextPaint.setTypeface(Typeface.MONOSPACE);
        targetTextPaint.setTextSize(20f);

        axisTextPaint.setColor(Color.DKGRAY);
        axisTextPaint.setTextAlign(Paint.Align.CENTER);
        axisTextPaint.setTextSize(28f);

        startZonePaint.setColor(Color.parseColor("#334CAF50")); // transparent green fill
        startZonePaint.setStyle(Paint.Style.FILL);

        // spacing (tweak if you want more/less)
        insetPx = 24f;      // outer spacing
        axisMarginPx = 56f; // room for axis labels

        obstacleIdTextPaint.setColor(Color.WHITE);
        obstacleIdTextPaint.setTextAlign(Paint.Align.CENTER);
        obstacleIdTextPaint.setTypeface(Typeface.DEFAULT_BOLD);
        obstacleIdTextPaint.setTextSize(28f);

        faceLinePaint.setColor(Color.RED);
        faceLinePaint.setStrokeWidth(6f);
        faceLinePaint.setStyle(Paint.Style.STROKE);

        // Gesture detector: tap vs long press
        gestureDetector = new GestureDetector(getContext(), new GestureDetector.SimpleOnGestureListener() {

            @Override
            public boolean onDown(MotionEvent e) {
                return true;
            }

            @Override
            public void onLongPress(MotionEvent e) {
                Integer hit = findObstacleAt(e.getX(), e.getY());
                if (hit != null) {
                    draggingId = hit;
                    isDragging = true;
                    getParent().requestDisallowInterceptTouchEvent(true);
                }
            }
            @Override
            public boolean onSingleTapUp(MotionEvent e) {
                Integer hit = findObstacleAt(e.getX(), e.getY());
                if (hit != null) {
                    if (obstacleTapListener != null) obstacleTapListener.onObstacleTapped(hit);
                    return true;
                }

                selectedCell = pointToCellModel(e.getX(), e.getY());
                invalidate();

                if (selectedCell != null && cellTapListener != null) {
                    cellTapListener.onCellTapped(selectedCell.x, selectedCell.y);
                }
                return true;
            }
        });
    }

    // ---------- Rendering ----------

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        computeGeometry();

        // background
        canvas.drawColor(Color.WHITE);

        float left = originX;
        float top = originY;
        float right = originX + GRID_SIZE * cellSizePx;
        float bottom = originY + GRID_SIZE * cellSizePx;

        // 1) Start zone (draw behind everything)
        RectF startRect = startZoneRectModel();
        canvas.drawRect(startRect, startZonePaint);

        // 2) Selected cell highlight
        if (selectedCell != null) {
            RectF r = cellRectModel(selectedCell.x, selectedCell.y);
            canvas.drawRect(r, selectedPaint);
        }

        // 3) Grid lines + border
        for (int i = 0; i <= GRID_SIZE; i++) {
            float x = left + i * cellSizePx;
            canvas.drawLine(x, top, x, bottom, gridPaint);

            float y = top + i * cellSizePx;
            canvas.drawLine(left, y, right, y, gridPaint);
        }
        canvas.drawRect(left, top, right, bottom, borderPaint);

        // 4) Axes labels (1..20, bottom-left origin)
        drawAxesBottomLeft(canvas, left, top, right, bottom);

        // 5) Obstacles (fill + id text + red face line)
        for (Obstacle o : obstacles.values()) {
            RectF rect = cellRectModel(o.getX(), o.getY());

            // obstacle fill
            canvas.drawRect(rect, obstaclePaint);

            // obstacle id in center
            float cx = rect.centerX();
            float cy = rect.centerY();
            canvas.drawText(
                    String.valueOf(o.getId()),
                    cx,
                    cy + obstacleIdTextPaint.getTextSize() / 3f,
                    obstacleIdTextPaint
            );

            // red face line
            drawFaceLine(canvas, rect, o.getFace());
        }
    }


    private void computeGeometry() {
        float w = getWidth();
        float h = getHeight();

        // Reserve space for axis labels around the grid
        float usableW = w - 2 * insetPx - axisMarginPx;
        float usableH = h - 2 * insetPx - axisMarginPx;

        cellSizePx = Math.min(usableW, usableH) / GRID_SIZE;
        float gridPx = cellSizePx * GRID_SIZE;

        // Origin shifts right and down to make room for Y-axis labels (left) and X-axis labels (bottom)
        originX = insetPx + axisMarginPx;
        originY = insetPx;

        // Center it within the remaining space (optional, looks nicer)
        float extraW = usableW - gridPx;
        float extraH = usableH - gridPx;

        originX += extraW / 2f;
        originY += extraH / 2f;
    }

    private RectF cellRect(int x, int y) {
        float left = originX + x * cellSizePx;
        float top = originY + y * cellSizePx;
        return new RectF(left, top, left + cellSizePx, top + cellSizePx);
    }

    private @Nullable Cell pointToCell(float px, float py) {
        int x = (int) ((px - originX) / cellSizePx);
        int y = (int) ((py - originY) / cellSizePx);
        if (x < 0 || x >= GRID_SIZE || y < 0 || y >= GRID_SIZE) return null;
        return new Cell(x, y);
    }

    private @Nullable Integer findObstacleAt(float px, float py) {
        for (Obstacle o : obstacles.values()) {
            if (cellRect(o.getX(), o.getY()).contains(px, py)) {
                return o.getId();
            }
        }
        return null;
    }

    // ---------- Touch handling (tap select + drag obstacles) ----------

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        gestureDetector.onTouchEvent(event);

        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_MOVE: {
                if (!isDragging || draggingId == null) return true;

                Cell c = pointToCellModel(event.getX(), event.getY());
                if (c != null) {
                    Obstacle o = obstacles.get(draggingId);
                    if (o != null) {
                        o.setX(c.x);
                        o.setY(c.y);
                    }
                }
                invalidate();
                return true;
            }

            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL: {
                if (isDragging && draggingId != null) {
                    Cell c = pointToCellModel(event.getX(), event.getY());
                    if (c == null) {
                        obstacles.remove(draggingId);
                    }
                    draggingId = null;
                    isDragging = false;
                    invalidate();
                }
                return true;
            }
        }
        return true;
    }
    // ---------- Public APIs (used by Fragment / Bluetooth layer) ----------

    public void upsertObstacle(int id, int x, int y, Obstacle.Dir face) {
        if (!inBounds(x, y)) return;

        Obstacle existing = obstacles.get(id);
        if (existing == null) {
            obstacles.put(id, new Obstacle(id, x, y, face));
        } else {
            existing.setX(x);
            existing.setY(y);
            existing.setFace(face);
        }
        invalidate();
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

    public void setTarget(int id, Integer targetId) {
        Obstacle o = obstacles.get(id);
        if (o != null) {
            o.setTargetId(targetId);
            invalidate();
        }
    }

    public void clearAll() {
        obstacles.clear();
        selectedCell = null;
        draggingId = null;
        invalidate();
    }

    // Expected: "ADD,1,5,10,N"
    // Also supports: "SUB,1"  "FACE,1,E"  "TARGET,1,11"
    public void applyCommand(String cmd) {
        if (cmd == null) return;
        String[] p = cmd.trim().split(",");
        if (p.length == 0) return;

        String op = p[0].trim().toUpperCase(Locale.US);

        switch (op) {
            case "ADD": {
                if (p.length < 5) return;
                Integer id = toInt(p[1]);
                Integer x = toInt(p[2]);
                Integer y = toInt(p[3]);
                Obstacle.Dir face = toDir(p[4]);
                if (id == null || x == null || y == null || face == null) return;
                upsertObstacle(id, x, y, face);
                break;
            }
            case "SUB":
            case "REMOVE": {
                if (p.length < 2) return;
                Integer id = toInt(p[1]);
                if (id == null) return;
                removeObstacle(id);
                break;
            }
            case "FACE": {
                if (p.length < 3) return;
                Integer id = toInt(p[1]);
                Obstacle.Dir face = toDir(p[2]);
                if (id == null || face == null) return;
                setObstacleFace(id, face);
                break;
            }
            case "TARGET": {
                if (p.length < 3) return;
                Integer id = toInt(p[1]);
                Integer target = toInt(p[2]); // allow null if parse fails
                if (id == null) return;
                setTarget(id, target);
                break;
            }
        }
    }

    private boolean inBounds(int x, int y) {
        return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
    }

    private @Nullable Integer toInt(String s) {
        try { return Integer.parseInt(s.trim()); }
        catch (Exception e) { return null; }
    }

    private @Nullable Obstacle.Dir toDir(String s) {
        if (s == null) return null;
        switch (s.trim().toUpperCase(Locale.US)) {
            case "N": return Obstacle.Dir.N;
            case "E": return Obstacle.Dir.E;
            case "S": return Obstacle.Dir.S;
            case "W": return Obstacle.Dir.W;
            default: return null;
        }
    }

    private static class Cell {
        final int x, y;
        Cell(int x, int y) { this.x = x; this.y = y; }
    }

    private RectF startZoneRect() {
        // bottom-left on screen means y is the last rows
        int startX0 = 0;
        int startY0 = GRID_SIZE - START_CELLS; // 20-4 = 16

        float left = originX + startX0 * cellSizePx;
        float top = originY + startY0 * cellSizePx;
        float right = left + START_CELLS * cellSizePx;
        float bottom = top + START_CELLS * cellSizePx;

        return new RectF(left, top, right, bottom);
    }

    public void setOnCellTapListener(OnCellTapListener listener) {
        this.cellTapListener = listener;
    }

    // Convert MODEL y (0 bottom) to SCREEN row (0 top)
    private int modelYToScreenRow(int modelY) {
        return (GRID_SIZE - 1) - modelY;
    }

    // Rect for a cell using MODEL coords
    private RectF cellRectModel(int x, int modelY) {
        int screenRow = modelYToScreenRow(modelY);

        float left = originX + x * cellSizePx;
        float top = originY + screenRow * cellSizePx;

        return new RectF(left, top, left + cellSizePx, top + cellSizePx);
    }

    // Convert touch point to MODEL cell (bottom-left origin)
    private @Nullable Cell pointToCellModel(float px, float py) {
        int x = (int) ((px - originX) / cellSizePx);
        int screenRow = (int) ((py - originY) / cellSizePx);

        if (x < 0 || x >= GRID_SIZE || screenRow < 0 || screenRow >= GRID_SIZE) return null;

        int modelY = (GRID_SIZE - 1) - screenRow;
        return new Cell(x, modelY);
    }

    // Start zone rect for MODEL coords: bottom-left 4x4 means x=0..3, y=0..3
    private RectF startZoneRectModel() {
        // bottom-left in MODEL coords
        int x0 = 0;
        int y0 = 0;

        // top-left corner in SCREEN space is model y=3 converted to screen row
        // easiest: build rect using 2 corners
        RectF bottomLeftCell = cellRectModel(x0, y0);
        RectF topRightCell = cellRectModel(START_CELLS - 1, START_CELLS - 1);

        float left = bottomLeftCell.left;
        float bottom = bottomLeftCell.bottom;

        float right = topRightCell.right;
        float top = topRightCell.top;

        return new RectF(left, top, right, bottom);
    }

    // Axes with (0,0) at bottom-left
    private void drawAxesBottomLeft(Canvas canvas, float left, float top, float right, float bottom) {

        // If you want: make axis numbers smaller so 20 labels fit nicely
        float oldSize = axisTextPaint.getTextSize();
        axisTextPaint.setTextSize(18f);

        // X-axis labels: 0..19 under each column
        for (int x = 0; x < GRID_SIZE; x++) {
            float cx = left + (x + 0.5f) * cellSizePx;
            float cy = bottom + 32f; // below grid
            canvas.drawText(String.valueOf(x), cx, cy, axisTextPaint);
        }
        canvas.drawText("X", (left + right) / 2f, bottom + 58f, axisTextPaint);

        // Y-axis labels: 0..19 along left side (0 at bottom)
        for (int y = 0; y < GRID_SIZE; y++) {
            int screenRow = modelYToScreenRow(y);
            float cx = left - 22f; // left of grid
            float cy = top + (screenRow + 0.5f) * cellSizePx + axisTextPaint.getTextSize() / 3f;
            canvas.drawText(String.valueOf(y), cx, cy, axisTextPaint);
        }
        canvas.drawText("Y", left - 50f, top - 12f, axisTextPaint);

        // restore size
        axisTextPaint.setTextSize(oldSize);
    }

    private void drawFaceLine(Canvas canvas, RectF rect, Obstacle.Dir face) {
        float pad = 6f;

        switch (face) {
            case N:
                canvas.drawLine(rect.left + pad, rect.top + pad, rect.right - pad, rect.top + pad, faceLinePaint);
                break;
            case E:
                canvas.drawLine(rect.right - pad, rect.top + pad, rect.right - pad, rect.bottom - pad, faceLinePaint);
                break;
            case S:
                canvas.drawLine(rect.left + pad, rect.bottom - pad, rect.right - pad, rect.bottom - pad, faceLinePaint);
                break;
            case W:
                canvas.drawLine(rect.left + pad, rect.top + pad, rect.left + pad, rect.bottom - pad, faceLinePaint);
                break;
        }
    }
}
