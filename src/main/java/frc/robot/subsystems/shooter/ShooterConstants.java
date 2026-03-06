package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final double SHOOTER_VELOCITY = 1.0;
    public static final double SHOOTER_LAUNCH_DIAMETER = Units.inchesToMeters(4.0);

    // for bang bang
    public static final double TORQUE_CURRENT_CONTROL_TOLERANCE = 10; // velocity (rotations per second)

    public static final double TORQUE_CURRENT_CONTROL_GOAL_AMP = 40; // TUNE
}
