package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.util.FieldZone;

public class FieldConstants {
  /** Width of the field [meters] */
  public static final double FIELD_LENGTH = Units.inchesToMeters(57*12 + 6+7.0/8.0);
  /** Height of the field [meters] */
  public static final double FIELD_WIDTH = Units.inchesToMeters(26*12 + 5);

  /**Apriltag layout for 2026 REBUILT */
  public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final double RED_BORDER = Units.inchesToMeters(180);
  public static final double BLUE_BORDER = FIELD_LENGTH - Units.inchesToMeters(180);
  public static final double LEFT_SIDE_TARGET = FIELD_WIDTH * 0.25;
  public static final double RIGHT_SIDE_TARGET = FIELD_WIDTH * 0.75;

  /** Location of hub target */
  public static final Translation3d HUB_BLUE =
      new Translation3d(Units.inchesToMeters(156.8 + 20), 4.035, Units.inchesToMeters(72));
  
  public static final Translation3d HUB_RED =
      new Translation3d(FIELD_LENGTH - Units.inchesToMeters(156.8 + 20), 4.035 + .67, Units.inchesToMeters(72));
    
  public static final Translation3d NEUTRAL_LEFT =
    new Translation3d(FIELD_LENGTH/2, LEFT_SIDE_TARGET, 0);

  public static final Translation3d NEUTRAL_RIGHT =
    new Translation3d(FIELD_LENGTH/2, RIGHT_SIDE_TARGET, 0);

  public static final Translation3d ALLIANCE_LEFT_BLUE =
    new Translation3d(BLUE_BORDER + 5, LEFT_SIDE_TARGET, 0); // previous hub + a few feet further back

  public static final Translation3d ALLIANCE_RIGHT_BLUE =
    new Translation3d(BLUE_BORDER + 5, RIGHT_SIDE_TARGET, 0);


  public static final Translation3d ALLIANCE_LEFT_RED =
    new Translation3d(RED_BORDER + 5, LEFT_SIDE_TARGET, 0); // previous hub + a few feet further back

  public static final Translation3d ALLIANCE_RIGHT_RED =
    new Translation3d(RED_BORDER + 5, RIGHT_SIDE_TARGET, 0);

  public static final double BlueAllianceLine = BLUE_BORDER; // That's the distance from one side to the blue bump
  public static final double RedAllianceLine = RED_BORDER; // 

  public static Translation3d getHubTranslation() {
    if (Robot.getAlliance() == Alliance.Blue) {
      return HUB_BLUE;
    } else {
      return HUB_RED;
    }
  }

  public static Translation3d getNeutralTranslation(boolean sideLeft) {
    if (sideLeft) {
      return NEUTRAL_LEFT;
    } else {
      return NEUTRAL_RIGHT;
    }
  }

  public static Translation3d getAllianceTranslation(boolean sideLeft) {
    if (sideLeft) {
      if (Robot.getAlliance() == Alliance.Blue) {
        return ALLIANCE_LEFT_BLUE;
      } else {
        return ALLIANCE_LEFT_RED;
      }
    } else {
      if (Robot.getAlliance() == Alliance.Blue) {
        return ALLIANCE_RIGHT_BLUE;
      } else {
        return ALLIANCE_RIGHT_RED;
      }
    }
  }

  public static FieldZone getZone(Translation2d drivepose) {
    double x = drivepose.getX();
    double y = drivepose.getY();
    if(x < FieldConstants.RedAllianceLine) { // inside red
      if (Robot.getAlliance() == Alliance.Red) {
        return FieldZone.ALLIANCE;
      } else {
        return FieldZone.OPPOSITION;
      }
    } else if (x > FieldConstants.BlueAllianceLine) {
      if (Robot.getAlliance() == Alliance.Blue) {
        return FieldZone.ALLIANCE;
      } else {
        return FieldZone.OPPOSITION;
      }
    } else {
      return FieldZone.NEUTRAL;
    }
  }

  public static Translation3d getOppositionTranslation(boolean sideLeft) {
    if (sideLeft) {
      if (Robot.getAlliance() == Alliance.Blue) {
        return ALLIANCE_LEFT_RED;
      } else {
        return ALLIANCE_LEFT_BLUE;
      }
    } else {
      if (Robot.getAlliance() == Alliance.Blue) {
        return ALLIANCE_RIGHT_RED;
      } else {
        return ALLIANCE_RIGHT_BLUE;
      }
    }
  }
}