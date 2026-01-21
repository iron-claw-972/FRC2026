package frc.robot.commands.drive_comm;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Vision.DriverAssist;

/**
 * faces forward, clone of defalut drive command
 */
public class FaceForward extends Command {
    private Drivetrain drive;
    private BaseDriverConfig driver;

    private double CONTROL_DEADZONE = 0.1;
    private double SPEED_DEADZONE = 1.0; //TODO change values

    @AutoLogOutput(key = "FaceForward/XVelocity")
    public double getXVelocity(){
        return drive.getChassisSpeeds().vxMetersPerSecond;
    }

    @AutoLogOutput(key = "FaceForward/YVelocity")
    public double getYVelocity(){
        return drive.getChassisSpeeds().vyMetersPerSecond;
    }

    @AutoLogOutput(key = "FaceForward/DesiredAngleDegs")
    public double getDesiredAngle(){
        Translation2d velocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
        Rotation2d desiredAngle = new Rotation2d(velocity.getX(), velocity.getY());

        return desiredAngle.getDegrees();
    }



    public FaceForward(Drivetrain drive, BaseDriverConfig driver) {
        this.drive = drive;
        this.driver = driver;
    }

    @Override
    public void initialize() {
        drive.setIsAlign(true);
        driver.getRawHeadingAngle();

        SmartDashboard.putNumber("controller deadzone", CONTROL_DEADZONE);
        SmartDashboard.putNumber("speed deadzone", SPEED_DEADZONE);
    }

    @Override
    public void execute() {
        CONTROL_DEADZONE = SmartDashboard.getNumber("controller deadzone", CONTROL_DEADZONE);
        SPEED_DEADZONE = SmartDashboard.getNumber("speed deadzone", SPEED_DEADZONE);        

        Translation2d robotRelativeVelocity = new Translation2d(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
        Rotation2d robotRelativeAngle = new Rotation2d(robotRelativeVelocity.getX(), robotRelativeVelocity.getY());

        //Rotation2d desiredAngle = robotRelativeAngle.rotateBy(drive.getYaw()).rotateBy(new Rotation2d(Math.PI/2));
        Rotation2d desiredAngle = new Rotation2d(-driver.getRawSideTranslation(), driver.getRawForwardTranslation()); //this feels better

        if (driver.getRawHeadingMagnitude() > CONTROL_DEADZONE) {
            drive.setAlignAngle(desiredAngle.getRadians());
        } // else if (robotRelativeVelocity.getNorm() > SPEED_DEADZONE){
        //     drive.setAlignAngle(desiredAngle.getRadians());
        // }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setIsAlign(false);
    }

}
