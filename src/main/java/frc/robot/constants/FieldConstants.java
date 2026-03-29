package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Zone;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.swerve.DriveConstants;

public class FieldConstants {

  /**Apriltag layout for 2026 REBUILT */
  public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /** Width of the field [meters] */
  public static final double FIELD_LENGTH = field.getFieldLength();
  /** Height of the field [meters] */
  public static final double FIELD_WIDTH = field.getFieldWidth();

  public static final double RED_BORDER = FIELD_LENGTH/2 + Units.inchesToMeters(167.0);
  public static final double BLUE_BORDER = FIELD_LENGTH/2 - Units.inchesToMeters(167.0);
  public static final double LEFT_SIDE_TARGET = FIELD_WIDTH * 0.167;
  public static final double RIGHT_SIDE_TARGET = FIELD_WIDTH * 0.833;

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
    
  // shuttling locations
  public static final Translation3d NEUTRAL_LEFT =
    new Translation3d(FIELD_LENGTH/2, LEFT_SIDE_TARGET, 0);

  public static final Translation3d NEUTRAL_RIGHT =
    new Translation3d(FIELD_LENGTH/2, RIGHT_SIDE_TARGET, 0);

  public static final Translation3d ALLIANCE_LEFT_BLUE =
    new Translation3d(BLUE_BORDER - 2.2, LEFT_SIDE_TARGET, 0); // previous hub + a few feet further back

  public static final Translation3d ALLIANCE_RIGHT_BLUE =
    new Translation3d(BLUE_BORDER - 2.2, RIGHT_SIDE_TARGET, 0);

  public static final Translation3d ALLIANCE_LEFT_RED =
    new Translation3d(RED_BORDER + 2.2, LEFT_SIDE_TARGET, 0); // previous hub + a few feet further back

  public static final Translation3d ALLIANCE_RIGHT_RED =
    new Translation3d(RED_BORDER + 2.2, RIGHT_SIDE_TARGET, 0);

  public static final Translation3d ALLIANCE_CENTER_BLUE =
    new Translation3d(BLUE_BORDER - 2, FIELD_WIDTH/2, 0);
  
  public static final Translation3d ALLIANCE_CENTER_RED =
    new Translation3d(RED_BORDER + 2, FIELD_WIDTH/2, 0);

  public static final double BLUE_ALLIANCE_LINE = BLUE_BORDER; // That's the distance from one side to the blue bump
  public static final double RED_ALLIANCE_LINE = RED_BORDER; // 

  // my zones
  public static final double leftNeutralLine = FIELD_LENGTH * 0.25;
  public static final double rightNeutralLine = FIELD_LENGTH * 0.75;
  public static final double centerLengthLine = FIELD_LENGTH * 0.5;
  public static final double centerWidthLine = FIELD_WIDTH * 0.5;
  public static final double redLine = 179.111250;
  public static final double blueLine = FIELD_LENGTH - 179.111250;

  public static final double hubWidthLeft = FIELD_WIDTH / 2 - (47.0 / 2);
  public static final double hubWidthRight = FIELD_WIDTH / 2 + (47.0 / 2);
  public static final double hubBackRed = FIELD_LENGTH + 120.0; 
  public static final double hubBackBlue = FIELD_LENGTH - 120.0; 

  public static final double ladderRedLeft = FIELD_WIDTH + 13.0;
  public static final double ladderBlueLeft = FIELD_WIDTH - 12.375;
  public static final double ladderRedRight = FIELD_WIDTH - 35.75;
  public static final double ladderBlueRight = FIELD_WIDTH + 35.75;

  public static final Zone neutralStrip = new Zone(centerLengthLine, centerWidthLine, rightNeutralLine - leftNeutralLine, redLine - blueLine);
  public static final Zone neutralLeft = new Zone(centerLengthLine, centerWidthLine, rightNeutralLine - leftNeutralLine, redLine - blueLine);
  public static final Zone neutralRight = new Zone(centerLengthLine, centerWidthLine, rightNeutralLine - leftNeutralLine, redLine - blueLine);
  public static final Zone blueHubOut = new Zone(centerLengthLine, centerWidthLine, rightNeutralLine - leftNeutralLine, redLine - blueLine);
  public static final Zone redHubOut = new Zone(centerLengthLine, centerWidthLine, rightNeutralLine - leftNeutralLine, redLine - blueLine);

  // trenches

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
		TRENCH_BUMP,
    UNDER_LADDER,
    UNDER_MY_HUB,
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
    double y = drivepose.getY();
    //double y = drivepose.getY();
    if ((x < FIELD_LENGTH/2 - Units.inchesToMeters(120.0) && x > (BLUE_ALLIANCE_LINE + (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS)/2)) //blue alliance line
        || x > FIELD_LENGTH/2 + Units.inchesToMeters(120.0) && x < (RED_ALLIANCE_LINE - (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS)/2)) {
          return FieldZone.TRENCH_BUMP;
        }
    // if(((y < (FIELD_WIDTH / 2) + 0.158750) && y > (FIELD_WIDTH / 2) - 0.736600) && (x < Units.inchesToMeters(47.0) || x > FIELD_LENGTH - Units.inchesToMeters(47.0)) && Robot.getAlliance() == Alliance.Blue) {
    //   return FieldZone.UNDER_LADDER;
    // }
    // if(((y < (FIELD_WIDTH / 2) - 0.158750) && y > (FIELD_WIDTH / 2) + 0.736600) && (x < Units.inchesToMeters(47.0) || x > FIELD_LENGTH - Units.inchesToMeters(47.0)) && Robot.getAlliance() == Alliance.Red) {
    //   return FieldZone.UNDER_LADDER;
    // }
    // if(((y < FIELD_WIDTH - (46.75 / 2) && y > FIELD_WIDTH - (46.75 / 2))) && ((x > 207.42375 && x < 207.42375 + 36.0) || (x < FIELD_LENGTH - 207.42375 && x > FIELD_LENGTH - 207.42375 - 36.0))) {
    //   return FieldZone.UNDER_LADDER;
    // }
    if(x > FieldConstants.RED_ALLIANCE_LINE - (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS)/2) { // inside red
      if (Robot.getAlliance() == Alliance.Red) {
        return FieldZone.ALLIANCE;
      } else {
        return FieldZone.OPPOSITION;
      }
    } else if (x < FieldConstants.BLUE_ALLIANCE_LINE + (DriveConstants.ROBOT_WIDTH_WITH_BUMPERS)/2) {
      if (Robot.getAlliance() == Alliance.Blue) {
        return FieldZone.ALLIANCE;
      } else {
        return FieldZone.OPPOSITION;
      }
    } else {
      return FieldZone.NEUTRAL;
    }
  }

  public static boolean underTrench(double x, double y) {
    // ensures we aren't in center channel
    if (y > Units.inchesToMeters(50.0) && y < FIELD_WIDTH - Units.inchesToMeters(50)) {
      return false;
    }
    // if our location is to far away from right underneath trench in terms of x
    // in between blue alliance trench
    if (!(x > Units.inchesToMeters(152.5) && x < Units.inchesToMeters(187.5)) && !(x < FIELD_LENGTH - Units.inchesToMeters(152.5) && x > FIELD_LENGTH - Units.inchesToMeters(187.5))) {
      return false;
    }
    return true;
  }
  
  /**
   * 
   * @return Whether Y coordinate is in the upper half (left side on blue alliance)
   */
  public static boolean isOnLeftSideOfField(Translation2d drivepose){
    return drivepose.getY() > FIELD_WIDTH/2;
  }

  public static Translation3d getOppositionTranslation(boolean sideLeft) {
    if (sideLeft) {
      if (Robot.getAlliance() == Alliance.Blue) {
        return ALLIANCE_LEFT_RED;
      } else {
        // Reversed it so we shoot same side, but probably need to change this
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
