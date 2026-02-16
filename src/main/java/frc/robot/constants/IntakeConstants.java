package frc.robot.constants;

public class IntakeConstants {

    // Motors (set actual ids)
    /** Intake extender right motor CAN ID */
    public static final int rightID = 1;
    /** Intake extender left motor CAN ID */
    public static final int leftID = 2;
    /** Intake roller motor CAN ID */
    public static final int rollerID = 3;

    /** Intake roller motor speed in range [-1, 1] */
    public static final double speed = 0.2;
    /** 12 tooth pinion driving 36 tooth driven gear */
    public static final double gearRatio = 36.0/12.0;
    /** radius (inches) of the rack gear which is a 10 tooth pinion at 10 DP */
    public static final double radiusRackPinion = 0.5;
    /** roller current limits */
    public static final double rCurrentLimits = 10.0;
    /**right and left motor current limits */
    public static final double extendCurrentLimits = 40.0;

    // Intake positions

    /** max extension in inches */
    public static final double maxExtension = 3.0; // 14.856; 
    /** starting point in inches */
    public static final double startingPoint = 0;
    /** rack pitch in teeth per inch of diameter (Diametral Pitch) DP = N teeth / Diameter in inches */
    public static final double rackPitch = 10;
}
