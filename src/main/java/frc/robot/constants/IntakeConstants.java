package frc.robot.constants;

public class IntakeConstants {
    public static final double PIVOT_GEAR_RATIO = 1/(18.0*18.0*10.0/54.0/60.0/38.0);  //38

    public static final double MASS = 1.5753263; // kilograms: mass of arm specifically
    public static final double CENTER_OF_MASS_LENGTH = 0.199608903192622; // meters
    public static final double LENGTH = 0.245750; // meters

    public static final double MOI = 0.0644; // kg*m^2
    public static final double MAX_VELOCITY = 15.0; // rad/s
    public static final double MAX_ACCELERATION = 100.0; // rad/s^2

    public static final double START_ANGLE = 101.7539148;
    public static final double INTAKE_ANGLE = 142.48;
    public static final double STOW_ANGLE = 107.0;

    //TODO: find this
    public static final double FLYWHEEL_SPEED = 1.0;

    //TODO: find this
    public static final double ABSOLUTE_OFFSET_ANGLE = (139.1748046875 - START_ANGLE + 30);

}