package frc.robot.util;

public record Zone(
		double x_center,
		double y_center,
		double width,
		double length) {

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
