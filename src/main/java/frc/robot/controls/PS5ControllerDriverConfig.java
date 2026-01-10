package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.commands.gpm.IntakeBall;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.shooterReal;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.HoodReal;
import frc.robot.subsystems.intake.IntakeReal;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Key for shooter + hood + intake controls
 *  SQUARE: Aim hood at target (left button)
 *  CIRCLE: Shoot at target (right button)
 *  TRIANGLE: Align to target (top button)
 *  CROSS: Intake ball (bottom button)
 */

// TODO: Add some sensor logic

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private HoodReal hood;
    private shooterReal shooter;
    private IntakeReal intake;
    private Command intakeBall;
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = ()->false;

    //Turn on for alignment to the tag
    private Pose2d alignmentPose = null;
    private double HOOD_SETPOINT = HoodConstants.START_ANGLE;
    int intakeInt = 1;

    public PS5ControllerDriverConfig(Drivetrain drive, HoodReal hood, shooterReal shooter, IntakeReal intake) {
        super(drive);
        this.hood = hood;
        this.shooter = shooter;
        this.intake = intake;
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
        

        if (intake != null && shooter != null){

            // driver.get(PS5Button.LEFT_TRIGGER).onFalse(
            //     new InstantCommand(()-> {
            //         hood.resetDueToSlippingError();
            //     })
            // );
            
            //shoots it
            driver.get(PS5Button.LB).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()-> shooter.setShooter(-ShooterConstants.SHOOTER_VELOCITY)),
                new WaitCommand(0.4),
                new InstantCommand(()-> shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER))
            )
            ).onFalse(
                new InstantCommand(()->{
                    shooter.deactivateShooterAndFeeder();
                })
            );

        }
        
        // aim hood and drive
        driver.get(PS5Button.SQUARE).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setAlignmentPose()),
                new ParallelCommandGroup(
                    new DriveToPose(getDrivetrain(), ()-> alignmentPose)
                    //new InstantCommand(()-> hood.setToCalculatedAngle(HoodConstants.INITIAL_VELOCTIY, HoodConstants.TARGET_HEIGHT, hood.calculateDistanceToTarget(alignmentPose)
                    )
                )
        );

        if(intake != null){
            driver.get(PS5Button.CROSS).onTrue(
                new InstantCommand(()->{
                    if(intakeBall != null && intakeBall.isScheduled()){
                        intakeBall.cancel();
                    }
                    else{
                        intakeBall = new IntakeBall(intake, shooter);
                        intakeBall.schedule();
                    }
                })
            );

            driver.get(PS5Button.CIRCLE).onTrue(
                new InstantCommand(()->{
                    intake.outtakeFlyWheel();
                })
            );
        }
        
        //Cancel commands
        // driver.get(PS5Button.RB).onTrue(new InstantCommand(()->{
        //     if(intake != null){
        //         intake.stopFlyWheel();
        //         intake.setSetpoint(IntakeConstants.STOW_ANGLE);
        //     }
        //     if(shooter != null){
        //         shooter.stopFeeder();
        //         shooter.stopShooter();
        //     }
        //     if(hood != null){
        //         hood.setSetpoint(HoodConstants.START_ANGLE);
        //     }
        //     getDrivetrain().setIsAlign(false);
        //     getDrivetrain().setDesiredPose(()->null);
        //     alignmentPose = null;
        //     CommandScheduler.getInstance().cancelAll();
        // }));
    }

    public void setAlignmentPose(){
        Translation2d drivepose = getDrivetrain().getPose().getTranslation();
        //Uses tag #17
        int tagNumber = 17;
        Translation2d tagpose = FieldConstants.APRIL_TAGS.get(tagNumber - 1).pose.toPose2d().getTranslation();
        double YDifference = tagpose.getY()-drivepose.getY();
        double XDifference = tagpose.getX()-drivepose.getX();
        double angle = Math.atan(YDifference/XDifference);
        alignmentPose = new Pose2d(drivepose.getX(), drivepose.getY(), new Rotation2d(angle));
        System.out.println("Alignment Angle: " + Units.radiansToDegrees(angle));
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
