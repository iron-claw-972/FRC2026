package frc.robot.constants;

import java.lang.reflect.Field;

import org.opencv.dnn.Net;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.util.FieldZone;
import frc.robot.util.ShootingTarget;

public class FieldConstants {
  /** Width of the field [meters] */
  public static final double FIELD_LENGTH = Units.inchesToMeters(57*12 + 6+7.0/8.0);
  /** Height of the field [meters] */
  public static final double FIELD_WIDTH = Units.inchesToMeters(26*12 + 5);

  /**Apriltag layout for 2026 REBUILT */
  public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);


  /** Location of hub target */
  public static final Translation3d HUB_BLUE =
      new Translation3d(Units.inchesToMeters(156.8 + 20), 4.035 + .67, Units.inchesToMeters(72));
    
  // TODO: Update all of this
  public static final Translation3d NEUTRAL_LEFT =
    new Translation3d(field.getFieldLength()*0.5, field.getFieldWidth()*0.25, 0);

  public static final Translation3d NEUTRAL_RIGHT =
    new Translation3d(field.getFieldLength()*0.5, field.getFieldWidth() - NEUTRAL_LEFT.getX(), 0);

  public static final Translation3d ALLIANCE_LEFT_BLUE =
    new Translation3d(156.8+20+50, field.getFieldWidth()*0.25, 0); // previous hub + a few feet further back

  public static final Translation3d ALLIANCE_RIGHT_BLUE =
    new Translation3d(158.8+20+50, field.getFieldWidth() - ALLIANCE_LEFT_BLUE.getX(), 0);

  public static final double BlueAllianceLine = FieldConstants.FIELD_LENGTH * 0.75;
  public static final double RedAllianceLine = FieldConstants.FIELD_LENGTH * 0.25;

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
        return new Translation3d(
          field.getFieldLength() - ALLIANCE_LEFT_BLUE.getX(),
          ALLIANCE_LEFT_BLUE.getY(),
          ALLIANCE_LEFT_BLUE.getZ()
        );
      }
    } else {
      if (Robot.getAlliance() == Alliance.Blue) {
        return ALLIANCE_RIGHT_BLUE;
      } else {
        return new Translation3d(
          field.getFieldLength() - ALLIANCE_LEFT_BLUE.getX(),
          ALLIANCE_RIGHT_BLUE.getY(),
          ALLIANCE_RIGHT_BLUE.getZ()
        );
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
        return new Translation3d(
          field.getFieldLength() - ALLIANCE_LEFT_BLUE.getX(),
          ALLIANCE_LEFT_BLUE.getY(),
          ALLIANCE_LEFT_BLUE.getZ()
        );
      } else {
        return ALLIANCE_LEFT_BLUE;
      }
    } else {
      if (Robot.getAlliance() == Alliance.Blue) {
        return new Translation3d(
          field.getFieldLength() - ALLIANCE_LEFT_BLUE.getX(),
          ALLIANCE_RIGHT_BLUE.getY(),
          ALLIANCE_RIGHT_BLUE.getZ()
        );
      } else {
        return ALLIANCE_RIGHT_BLUE;
      }
    }
  }
}