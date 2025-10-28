package frc.robot.constants;

public class IntakeConstants {
    //TODO: Update these constants
    public static final double PIVOT_GEAR_RATIO = 1/(18.0*18.0*10.0/54.0/60.0/38.0);

    public static final double MASS = 2.46; // kilograms
    public static final double MOI = 0.0261057394; // kg*m^2
    public static final double CENTER_OF_MASS_LENGTH = 0.138; // meters
    public static final double LENGTH = CENTER_OF_MASS_LENGTH * 2; // meters
    public static final double MAX_VELOCITY = 21; // rad/s
    public static final double MAX_ACCELERATION = 120; // rad/s^2

    public static final double STOW_ANGLE = 90;
    public static final double START_ANGLE = STOW_ANGLE;
    //TODO: find this
    public static final double INTAKE_ANGLE = 40;

    //TODO: find this
    public static final double FLYWHEEL_SPEED = 0.8;
}