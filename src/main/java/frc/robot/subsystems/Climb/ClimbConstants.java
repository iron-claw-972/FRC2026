package frc.robot.subsystems.Climb;

import edu.wpi.first.math.util.Units;

public class ClimbConstants {

    // CHANGE LATER
    // gear ratio for converting motor rotations to linear distance
    public final static double CLIMB_GEAR_RATIO = 45.0;
    public final static double WHEEL_RADIUS = Units.inchesToMeters(0.334);
    public final static double BOTTOM_POSITION = Units.inchesToMeters(-8);
    public final static double CLIMB_POSITION = Units.inchesToMeters(-6);
    public final static double UP_POSITION = 0.0;

    // current limits (in amps)
    // CALIBRATION: Low current while finding hardstop to prevent damage
    // NORMAL: Moderate current for PID-controlled movement
    // CLIMB: High current for full-power climbing
    public final static double CALIBRATION_CURRENT = 7.0;
    public final static double CLIMB_CURRENT = 42.0;

    // PID Constants
    public final static double PID_P = 0.1;
    public final static double PID_I = 0.0;
    public final static double PID_D = 0.0;
    public final static double PID_TOLERANCE = 0.2;

    // Motor Limits
    public final static double MIN_POWER = -0.2;
    public final static double MAX_POWER = 0.2;
    public final static double CALIBRATION_POWER = 0.15;

    // Calibration
    public final static int CALIBRATION_COUNTER_LIMIT = 250;
}
