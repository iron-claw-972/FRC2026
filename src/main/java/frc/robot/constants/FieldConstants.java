package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

public class FieldConstants {

  /**Apriltag layout for 2026 REBUILT */
  public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /** Width of the field [meters] */
  public static final double FIELD_LENGTH = field.getFieldLength();
  /** Height of the field [meters] */
  public static final double FIELD_WIDTH = field.getFieldWidth();

  public static final double RED_BORDER = FIELD_LENGTH/2 + Units.inchesToMeters(167.0);
  public static final double BLUE_BORDER = FIELD_LENGTH/2 - Units.inchesToMeters(167.0);
  public static final double LEFT_SIDE_TARGET = FIELD_WIDTH * 0.25;
  public static final double RIGHT_SIDE_TARGET = FIELD_WIDTH * 0.75;

  /**The coordinate of the climb position */
  public static final Pose2d BLUE_CLIMB_LOCATION = new Pose2d(1.5, FIELD_WIDTH/2 - 2.0, new Rotation2d()); // TODO: find this
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
      new Translation3d(Units.inchesToMeters(182.11), FIELD_WIDTH/2, Units.inchesToMeters(72));
  
  public static final Translation3d HUB_RED =
      new Translation3d(FIELD_LENGTH - Units.inchesToMeters(182.11), FIELD_WIDTH/2, Units.inchesToMeters(72));
    
  public static final Translation3d NEUTRAL_LEFT =
    new Translation3d(FIELD_LENGTH/2, LEFT_SIDE_TARGET, 0);

  public static final Translation3d NEUTRAL_RIGHT =
    new Translation3d(FIELD_LENGTH/2, RIGHT_SIDE_TARGET, 0);

  public static final Translation3d ALLIANCE_LEFT_BLUE =
    new Translation3d(BLUE_BORDER - 2, LEFT_SIDE_TARGET, 0); // previous hub + a few feet further back

  public static final Translation3d ALLIANCE_RIGHT_BLUE =
    new Translation3d(BLUE_BORDER - 2, RIGHT_SIDE_TARGET, 0);


  public static final Translation3d ALLIANCE_LEFT_RED =
    new Translation3d(RED_BORDER + 2, LEFT_SIDE_TARGET, 0); // previous hub + a few feet further back

  public static final Translation3d ALLIANCE_RIGHT_RED =
    new Translation3d(RED_BORDER + 2, RIGHT_SIDE_TARGET, 0);

  public static final Translation3d ALLIANCE_CENTER_BLUE =
    new Translation3d(BLUE_BORDER - 2, FIELD_WIDTH/2, 0);
  
  public static final Translation3d ALLIANCE_CENTER_RED =
    new Translation3d(RED_BORDER + 2, FIELD_WIDTH/2, 0);

  public static final double BLUE_ALLIANCE_LINE = BLUE_BORDER; // That's the distance from one side to the blue bump
  public static final double RED_ALLIANCE_LINE = RED_BORDER; // 

	public enum ShootingTarget {
		HUB,
		NEUTRAL,
		ALLIANCE,
		OPPOSITION, // not sure why you'd ever do this :)
	}

	public enum FieldZone {
		ALLIANCE,
		NEUTRAL,
		OPPOSITION,
		TRENCH_BUMP
	}

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

  public static Translation3d getAllianceSideTranslation(boolean sideLeft) {
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

  public static Translation3d getAllianceCenterTranslation() {
    if (Robot.getAlliance() == Alliance.Blue) {
      return ALLIANCE_CENTER_BLUE;
    } else {
      return ALLIANCE_CENTER_RED;
    }
  }

  public static FieldZone getZone(Translation2d drivepose) {
    double x = drivepose.getX();
    //double y = drivepose.getY();
    if ((x < FIELD_LENGTH/2 - Units.inchesToMeters(120.0) && x > BLUE_ALLIANCE_LINE)
        || x > FIELD_LENGTH/2 + Units.inchesToMeters(120.0) && x < RED_ALLIANCE_LINE){
          return FieldZone.TRENCH_BUMP;
        }
    if(x > FieldConstants.RED_ALLIANCE_LINE) { // inside red
      if (Robot.getAlliance() == Alliance.Red) {
        return FieldZone.ALLIANCE;
      } else {
        return FieldZone.OPPOSITION;
      }
    } else if (x < FieldConstants.BLUE_ALLIANCE_LINE) {
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
        // Reversed it so we shoot same side, but probably need to change this
        return ALLIANCE_RIGHT_BLUE;
      }
    } else {
      if (Robot.getAlliance() == Alliance.Blue) {
        return ALLIANCE_RIGHT_RED;
      } else {
        return ALLIANCE_LEFT_BLUE;
      }
    }
  }
}
