package com.ntu.group24.android.models;

public class Obstacle {
    public enum Dir { N, E, S, W }

    private int id;
    private int x;          // 0..19
    private int y;          // 0..19
    private Dir face;       // N/E/S/W
    private Integer targetId; // nullable

    public Obstacle(int id, int x, int y, Dir face) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.face = face;
        this.targetId = null;
    }

    public int getId() { return id; }
    public int getX() { return x; }
    public int getY() { return y; }
    public Dir getFace() { return face; }
    public Integer getTargetId() { return targetId; }

    public void setX(int x) { this.x = x; }
    public void setY(int y) { this.y = y; }
    public void setFace(Dir face) { this.face = face; }
    public void setTargetId(Integer targetId) { this.targetId = targetId; }
}
