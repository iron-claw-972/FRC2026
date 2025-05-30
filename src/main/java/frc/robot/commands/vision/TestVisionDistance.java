package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.Vision;

/**
 * Gathers data on the distance limits of the camera used for vision.
 */
public class TestVisionDistance extends Command {
  private final Drivetrain drive;
  private final Vision vision;
  private Translation2d visionStartTranslation, driveStartTranslation;
  private Pose2d currentPose = null;
  private double driveDistance;
  private double visionDistance;

  private double speed;

  private final Timer endTimer = new Timer();
  private final Timer printTimer = new Timer();

  // How many seconds of not seeing april tag before ending the command
  private static final double END_DELAY = 0.25;

  // How many seconds between each data print
  private static final double PRINT_DELAY = 1;

  /**
   * Constructor for TestVisionDistance
   * @param speed What speed to move at, negative if backward
   * @param drive The drivetrain
   * @param vision The vision
   */
  public TestVisionDistance(double speed, Drivetrain drive, Vision vision){
    addRequirements(drive);
    this.drive = drive;
    this.speed = speed;
    this.vision = vision;
  }

  /**
   * Starts the timers and disables vision for odometry
   */
  @Override
  public void initialize() {

    endTimer.reset();
    printTimer.restart();

    drive.setVisionEnabled(false);

    currentPose = vision.getPose2d(drive.getPose());
    visionStartTranslation = currentPose.getTranslation();
    driveStartTranslation = drive.getPose().getTranslation();
    driveDistance = 0;
    visionDistance = 0;
  }

  /**
   * Drives the robot, finds the pose from the drivetrain and vision, and someimes prints the distances
   */
  @Override
  public void execute() {
    drive.drive(speed, 0, 0, false, false);
    Pose2d newestPose = vision.getPose2d(currentPose, drive.getPose());

    // If the camera can see the apriltag
    if (newestPose != null) {
      //update current pose
      currentPose = newestPose;
      // reset the timer
      endTimer.reset();
      driveDistance = drive.getPose().getTranslation().getDistance(driveStartTranslation);
      visionDistance = currentPose.getTranslation().getDistance(visionStartTranslation);
      SmartDashboard.putNumber("Vision test drive distance", driveDistance);
      SmartDashboard.putNumber("Vision test vision distnace", visionDistance);
      SmartDashboard.putNumber("Vision test error", visionDistance - driveDistance);
      SmartDashboard.putNumber("Vision test % error", (visionDistance-driveDistance) / driveDistance * 100);
      // If kPrintDelay seconds have passed, print the data
      if (printTimer.advanceIfElapsed(PRINT_DELAY)) {
        System.out.printf("\nDrive dist: %.2f\nVision dist: %.2f\nError: %.2f\n %% error: %.2f\n",
          driveDistance, visionDistance,
          visionDistance-driveDistance, (visionDistance-driveDistance) / driveDistance * 100
        );
      }
    } else {
      endTimer.start();
    }
  }

  /**
   * Re-enables vision and stops the robot
   */
  @Override
  public void end(boolean interrupted) {
    drive.setVisionEnabled(true);
    drive.stop();
  }

  /**
   * Returns if the command is finished
   * @return If the end delay has elapsed
   */
  @Override
  public boolean isFinished() {
    return endTimer.hasElapsed(END_DELAY);
  }
}