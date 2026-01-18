package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  /** Width of the field [meters] */
  public static final double FIELD_LENGTH = Units.inchesToMeters(57*12 + 6+7.0/8.0);
  /** Height of the field [meters] */
  public static final double FIELD_WIDTH = Units.inchesToMeters(26*12 + 5);

  public static final Translation3d HUB_TRANSLATION3D = new Translation3d(Units.inchesToMeters(156.8), 4.035, Units.inchesToMeters(72));

}
