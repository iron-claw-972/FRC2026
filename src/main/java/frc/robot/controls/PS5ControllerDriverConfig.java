package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.commands.gpm.MoveHood;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.shooterReal;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.HoodBase;
import frc.robot.subsystems.hood.HoodReal;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private HoodReal hood;
    private shooterReal shooter;
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = ()->false;

    //Turn on for alignment to the tag
    private boolean alignTrue = false;
    private Pose2d alignmentPose = null;
    private double HOOD_SETPOINT = HoodConstants.START_ANGLE;

    public PS5ControllerDriverConfig(Drivetrain drive, HoodReal hood, shooterReal shooter) {
        super(drive);
        this.hood = hood;
        this.shooter = shooter;
    }

    public void configureControls() {
        // Reset the yaw. Mainly useful for testing/driver practice
        driver.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
            new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI)
        )));

        // Cancel commands
        driver.get(PS5Button.RIGHT_TRIGGER).onTrue(new InstantCommand(()->{
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(()->null);
            CommandScheduler.getInstance().cancelAll();
        }));

        // Align wheels
        driver.get(PS5Button.MUTE).onTrue(new FunctionalCommand(
            ()->getDrivetrain().setStateDeadband(false),
            getDrivetrain()::alignWheels,
            interrupted->getDrivetrain().setStateDeadband(true),
            ()->false, getDrivetrain()).withTimeout(2));
        
        // Align and shoot
        driver.get(PS5Button.RIGHT_TRIGGER).onTrue(
            new SequentialCommandGroup(
                alignTrue ? new DriveToPose(getDrivetrain(), ()-> alignmentPose) : new DoNothing(), // TODO: Does this work?
                new MoveHood(hood, HOOD_SETPOINT),
                new InstantCommand(()-> {
                    shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER);
                    shooter.setShooter(ShooterConstants.SHOOTER_RUN_POWER);
                })
            )
        ).onFalse(
            new InstantCommand(()->{
                shooter.stopFeeder();
                shooter.stopShooter();
            })
        );
    }

    public void setAlignmentPose(){
        Translation2d drivepose = getDrivetrain().getPose().getTranslation();
        //Uses tag #18
        int tagNumber = 18;
        Translation2d tagpose = FieldConstants.APRIL_TAGS.get(tagNumber - 1).pose.toPose2d().getTranslation();
        double YDifference = tagpose.getY()-drivepose.getY();
        double XDifference = tagpose.getX()-drivepose.getX();
        double angle = Math.atan(YDifference/XDifference);
        alignmentPose = new Pose2d(drivepose.getX(), drivepose.getY(), new Rotation2d(angle));
    }
    
    @Override
    public double getRawSideTranslation() {
        return driver.get(PS5Axis.LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return driver.get(PS5Axis.LEFT_Y);
    }

    @Override
    public double getRawRotation() {
        return driver.get(PS5Axis.RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(driver.get(PS5Axis.RIGHT_X), -driver.get(PS5Axis.RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return Math.hypot(driver.get(PS5Axis.RIGHT_X), driver.get(PS5Axis.RIGHT_Y));
    }

    @Override
    public boolean getIsSlowMode() {
        return slowModeSupplier.getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return false;
    }

    public void startRumble(){
        driver.rumbleOn();
    }

    public void endRumble(){
        driver.rumbleOff();
    }
}
