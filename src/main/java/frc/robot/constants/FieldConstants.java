package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

public class FieldConstants {
  /** Width of the field [meters] */
  public static final double FIELD_LENGTH = Units.inchesToMeters(57*12 + 6+7.0/8.0);
  /** Height of the field [meters] */
  public static final double FIELD_WIDTH = Units.inchesToMeters(26*12 + 5);

  /**Apriltag layout for 2026 REBUILT */
  public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // TODO: Find this
  /**The coordinate of the climb position */
  public static final Pose2d BLUE_CLIMB_LOCATION = new Pose2d(0, 0, new Rotation2d());
  public static final Pose2d RED_CLIMB_LOCATION = new Pose2d(0, 0, new Rotation2d());

  public static final Pose2d getClimbLocation(){
    if (Robot.getAlliance() == Alliance.Blue){
      return BLUE_CLIMB_LOCATION;
    }
    else{
      return RED_CLIMB_LOCATION;
    }
  }

  /** Location of hub target */
  public static final Translation3d HUB_BLUE =
      new Translation3d(Units.inchesToMeters(156.8), 4.035, Units.inchesToMeters(72));

  public static Translation3d getHubTranslation() {
    if (Robot.getAlliance() == Alliance.Blue) {
      return HUB_BLUE;
    } else {
      return new Translation3d(
          field.getFieldLength() - HUB_BLUE.getX(),
          HUB_BLUE.getY(),
          HUB_BLUE.getZ()
      );
    }
  }
}