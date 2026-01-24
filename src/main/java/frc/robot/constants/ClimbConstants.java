package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ClimbConstants {    
    public static final double gear_ratio = 320.0;

    public static final double startAngle = 0.5; // Rotations
    public static final double absoluteOffsetAngle = 0.0;

    public static final double climbFirstStage = 200.18; // Degrees
    public static final double climbSecondStage = 180.0; // Degrees

    public static final double tolerance = 3.0;

    public static final double MAX_VELOCITY = Units.degreesToRotations(30);
    public static final double MAX_ACCELERATION = Units.degreesToRotations(150);
}
