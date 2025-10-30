package frc.robot.constants;

public class IntakeConstants {
    //TODO: Update these constants
    public static final double PIVOT_GEAR_RATIO = 1/(18.0*18.0*10.0/54.0/60.0/38.0);  //38

    public static final double MASS = 1.5753263; // kilograms: mass of arm specifically
    public static final double CENTER_OF_MASS_LENGTH = 0.199608903192622; // meters
    public static final double LENGTH = CENTER_OF_MASS_LENGTH * 2; // meters

    public static final double MOI = MASS * LENGTH; // kg*m^2
    public static final double MAX_VELOCITY = 2; // rad/s
    public static final double MAX_ACCELERATION = 10; // rad/s^2

    public static final double START_ANGLE = 101.7539148;
    public static final double END_ANGLE = 153.7635904;
}