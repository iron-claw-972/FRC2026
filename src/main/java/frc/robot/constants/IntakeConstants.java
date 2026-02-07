package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {

    // Motors (set actual ids)
    public static final int rightID = 1;
    public static final int leftID = 2;
    public static final int rollerID = 3;

    // Intake positions

    public static final double maxExtension = 12; // in inches: convert to rotations later
    public static final double startingPoint = 0;

    // for simulation
    public static final double kMaxRotations = 37.5;
    public static final double kMaxVisualLength = 0.75;
}
