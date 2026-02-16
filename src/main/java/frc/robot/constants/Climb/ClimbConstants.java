package frc.robot.constants.Climb;

public class ClimbConstants {
    
    // CHANGE LATER
    public final static double CLIMB_GEAR_RATIO = 9.0 / 1 * 5.0 / 1;
    public final static double MAX_VELOCITY = 1;
    public final static double MAX_ACCELERATION = 0.3;
    public final static double RADIUS = 0.3;
    public final static double CLIMB_HEIGHT = 4;
    public final static double STRONG_CURRENT = 42.0;
    public final static double WEAK_CURRENT = 7.0;
    public final static double OFFSET = 100.0;
    public final static double CLIMB_OFFSET = 80.0;

    // PID Constants
    public final static double PID_P = 0.1;
    public final static double PID_I = 0.0;
    public final static double PID_D = 0.0;
    public final static double PID_TOLERANCE = 0.2;

    // Motor Limits
    public final static double DEFAULT_CURRENT_LIMIT = 5.0;
    public final static double MIN_POWER = -0.2;
    public final static double MAX_POWER = 0.2;
    public final static double CALIBRATION_POWER = 0.15;

    // Calibration
    public final static int CALIBRATION_COUNTER_LIMIT = 250;
    public final static double CALIBRATION_POSITION_OFFSET = 1.0;
}
