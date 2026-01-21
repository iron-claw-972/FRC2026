package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Aims the robot at the closest April tag
 */
public class AimAtTag extends Command {
  private Drivetrain drive;
  private PIDController pid;

  /**
   * Aims the robot at the closest April tag
   * @param drive The drivetrain
   */
  public AimAtTag(Drivetrain drive){
    this.drive = drive;
    // Copy drive PID and changetolerance
    pid = new PIDController(
      drive.getRotationController().getP(),
      drive.getRotationController().getI(),
      drive.getRotationController().getD()
    );
    pid.setTolerance(Units.degreesToRadians(1));
    addRequirements(drive);
  }

  /**
   * Gets the closest tag and sets the setpoint to aim at it
   */
  @Override
  public void initialize(){
    double dist = Double.POSITIVE_INFINITY;
    Translation2d closest = new Translation2d();
    Translation2d driveTranslation = drive.getPose().getTranslation();
    for(AprilTag tag : FieldConstants.field.getTags()){
      Translation2d translation = tag.pose.toPose2d().getTranslation();
      double dist2 = driveTranslation.getDistance(translation);
      if(dist2 < dist){
        dist = dist2;
        closest = translation;
      }
    }
    pid.reset();
    pid.setSetpoint(Math.atan2(closest.getY() - driveTranslation.getY(), closest.getX() - driveTranslation.getX()));
  }
  
  /**
   * Runs the PID
   */
  @Override
  public void execute() {
    double angle = drive.getPose().getRotation().getRadians();
    // If the distance between the angles is more than 180 degrees, use an identical angle ±360 degrees
    if(angle - pid.getSetpoint() > Math.PI){
      angle -= 2*Math.PI;
    }else if(angle - pid.getSetpoint() < -Math.PI){
      angle += 2*Math.PI;
    }
    double speed = pid.calculate(angle);
    drive.drive(0, 0, speed, true, false);
  }

  /**
   * Stops the drivetrain
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  /**
   * Returns if the command is finished
   * @return If the PID is at the setpoint
   */
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}

