package frc.robot.commands.gpm;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;

/**
 * Aims the robot at the closest April tag
 */
public class AimAtPose extends Command {
  private Turret turret;
  private Drivetrain drive;
  private Pose2d target;

  /**
   * Aims the robot at the closest April tag
   * @param drive The drivetrain
   */
  public AimAtPose(Drivetrain drive, Turret turret, Pose2d target){
    this.turret = turret;
    this.drive = drive;
    this.target = target;
    addRequirements(turret);
  }

  /**
   * Gets the closest tag and sets the setpoint to aim at it
   */
  @Override
  public void initialize(){}
  
  @Override
  public void execute() {
    //Logger.recordOutput("TurretPose", new Pose2d(drive.getPose().getTranslation(), turret.getPosition()));
    //.setSetpoint(-target.getTranslation().minus(drive.getPose().getTranslation()).getAngle().getDegrees(), 0);
  }

  /**
   * Stops the drivetrain
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted) {
    
  }

  /**
   * Returns if the command is finished
   * @return If the PID is at the setpoint
   */
  @Override
  public boolean isFinished() {
    // return turret.atSetPoint();
    return false;
  }
}

