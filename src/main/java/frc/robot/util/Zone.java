package frc.robot.util;

public class Zone {
    public double x_center;
    public double y_center;
    public double width;
    public double length;

    public Zone(double x_center, double y_center, double width, double length) {
        this.x_center = x_center;
        this.y_center = y_center;
        this.width = width;
        this.length = length;
    }

    public boolean isInside(double x, double y) {
        if (x > x_center + (length / 2) || x < x_center - (length / 2)) {
            return false;
        }
        if (y > y_center + (width / 2) || y < y_center - (width / 2)) {
            return false;
        }
        return true;
    }
}
