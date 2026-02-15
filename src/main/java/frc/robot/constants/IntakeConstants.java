package frc.robot.constants;

public class IntakeConstants {

    // Motors (set actual ids)
    public static final int rightID = 1;
    public static final int leftID = 2;
    public static final int rollerID = 3;

    //Motor speed
    public static final double speed = 0.2;
    /**
     * 12 tooth pinion driving 36 tooth driven gear
     */
    public static final double gearRatio = 36.0/12.0;
    /**
     * radius of the rack gear which is a 10 tooth pinion
     */
    public static final double radius = 0.5;

    // Intake positions

    /**
     * max extension in inches
     */
    public static final double maxExtension = 3.0; // 14.856; 
    /**
     * starting point in inches
     */
    public static final double startingPoint = 0;
    /**
     * rack pitch in teeth per inch
     */
    public static final double rackPitch = 10;
    // for simulation
    public static final double kMaxRotations = 37.5;
}
