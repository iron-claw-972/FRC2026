package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    /** Intake roller motor speed in range [-1, 1] */
    public static final double SPEED = 1.0;
    /** 12 tooth pinion driving 36 tooth driven gear */
    public static final double GEAR_RATIO = 36.0/12.0;
    /** radius (inches) of the rack gear which is a 10 tooth pinion at 10 DP */
    public static final double RADIUS_RACK_PINION = 0.5;
    /**right and left motor current limits */
    public static final double EXTENDER_CURRENT_LIMITS = 40.0;

    public static final double ROLLER_MOI_KG_M_SQ = 0.5 * 0.020 * 0.020; // 0.5kg roller, 20mm radius for now
    public static final double ROLLER_GEARING = 2.0;
    public static final double CARRIAGE_MASS_KG = 3.0;
    public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);


    /** max extension in inches */
    public static final double MAX_EXTENSION = 10.0; // inches

    public static final double INTERMEDIATE_EXTENSION = 5.0; //inches

    public static final double STOW_EXTENSION = 0.2; // inches

    /** starting point in inches */
    public static final double STARTING_POINT = 0;
    /** rack pitch in teeth per inch of diameter (Diametral Pitch) DP = N teeth / Diameter in inches */
    public static final double RACK_PITCH = 10;

    // Simulation 

}
