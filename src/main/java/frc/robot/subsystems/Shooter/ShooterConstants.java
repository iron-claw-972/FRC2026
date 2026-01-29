package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation3d;

public class ShooterConstants {
	// TODO: find these values
	public static final double FEEDER_RUN_POWER = 0.3; // percentage power
	public static final double SHOOTER_VELOCITY = 8.0;
	// meters per second
	public static final double SHOOTER_GEAR_RATIO = 36.0 / 24.0; // gear ratio from motors to shooter wheel
	public static final double SHOOTER_WHEEL_DIAMETER = 0.1016; // meters (4 inches)
	public static final double SHOOTER_RUN_POWER = 0.3;
	public static final double SENSOR_DISTANCE_THRESHOLD = 0.150; // meters
	public static final double SENSOR_AMBIENCE_THRESHOLD = 0.100; // meters

	// the transform from the center of the robot on the floor to the outtake
	// TODO: find this
	public static final Translation3d SHOOTER_TRANSFORM = Translation3d.kZero;
}
// 8 velcocity is too little
// 16 is too much
