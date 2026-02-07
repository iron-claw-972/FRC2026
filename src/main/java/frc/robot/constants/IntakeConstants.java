package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {

    // Motors (set actual ids)
    public static final int rightID = 1;
    public static final int leftID = 2;
    public static final int rollerID = 3;

    //Motor speed
    public static final double speed = 0.2;
    public static final double gearRatio = 3;
    public static final double radius = 0.5;

     public static final double statorLimitAmps = 50.0;
    public static final double supplyLimitAmps = 40.0;

    // Intake positions

    public static final double maxExtension = 12; // in inches: convert to rotations later
    public static final double startingPoint = 0;

    public static final double rackPitch = 10;
    // for simulation
    public static final double kMaxRotations = 37.5;
}
