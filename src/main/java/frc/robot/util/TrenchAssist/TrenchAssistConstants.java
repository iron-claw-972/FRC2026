package frc.robot.util.TrenchAssist;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class TrenchAssistConstants {
    public static final Rectangle2d[] OBSTACLES = new Rectangle2d[]{
        new Rectangle2d(new Translation2d(4.03, 1.28), new Translation2d(5.22, 1.58)),
        new Rectangle2d(new Translation2d(4.03, 8.07 - 1.28), new Translation2d(5.22, 8.07 - 1.58)),
        new Rectangle2d(new Translation2d(11.32, 1.28), new Translation2d(12.51, 1.58)),
        new Rectangle2d(new Translation2d(11.32, 8.07 - 1.28), new Translation2d(12.51, 8.07 - 1.58)),
    }; //8.07m

    public static final Rectangle2d[] ALIGN_ZONES = new Rectangle2d[]{
        new Rectangle2d(new Translation2d(4.03 - .5, 0), new Translation2d(5.22 + .5, 3)),
        new Rectangle2d(new Translation2d(4.03 - .5, 8.07), new Translation2d(5.22 + .5, 8.07 - 3)),
        new Rectangle2d(new Translation2d(11.32 - .5, 0), new Translation2d(12.51 + .5, 3)),
        new Rectangle2d(new Translation2d(11.32 - .5, 8.07), new Translation2d(12.51 + .5, 8.07 - 3)),
        new Rectangle2d(new Translation2d(99.99, 99.9), new Translation2d(0.0, 0.0)),
    };

    public static final double[] SLIDE_LATITUDES = new double[]{
        // 8.07 - Units.inchesToMeters(30.0),
        // Units.inchesToMeters(30.0), should be accurate, i think our field is slightly too small
        6.550,
        0.668,

    };

}