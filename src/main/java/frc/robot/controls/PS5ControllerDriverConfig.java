package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.commands.gpm.IntakeBall;
import frc.robot.commands.gpm.MoveHood;
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
 *  SQUARE: Aim hood at target
 *  CIRCLE: Shoot at target
 *  TRIANGLE: Align to target
 *  CROSS: Intake ball
 */

// TODO: Add some sensor logic

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private HoodReal hood;
    private shooterReal shooter;
    private IntakeReal intake;
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = ()->false;

    //Turn on for alignment to the tag
    private boolean alignTrue = true;
    private Pose2d alignmentPose = null;
    private double HOOD_SETPOINT = HoodConstants.START_ANGLE;

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
            // Align and shoot
            driver.get(PS5Button.RIGHT_TRIGGER).onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(()-> setAlignmentPose()),
                    new ParallelCommandGroup(
                        alignTrue ? new DriveToPose(getDrivetrain(), ()-> alignmentPose) : new DoNothing(),
                        //new MoveHood(hood, HOOD_SETPOINT)
                        new InstantCommand(()-> shooter.setShooter(ShooterConstants.SHOOTER_RUN_POWER))
                    ),
                    new WaitCommand(0.5),
                    new InstantCommand(()-> shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER))
                )
            ).onFalse(
                new InstantCommand(()-> {
                    shooter.deactivateShooterAndFeeder();
                })
            );

            // aim hood for shooter
            driver.get(PS5Button.SQUARE).onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(()-> setAlignmentPose()),
                    new ParallelCommandGroup(
                        new DriveToPose(getDrivetrain(), ()-> alignmentPose),
                        new InstantCommand(() -> hood.aimToTarget(getDrivetrain().getPose()))
                    )
                )
            );
  
            driver.get(PS5Button.LEFT_TRIGGER).onFalse(
                new InstantCommand(()-> {
                    hood.resetDueToSlippingError();
                })
            );
            
            // shoots it
            driver.get(PS5Button.CIRCLE).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()-> shooter.setShooter(ShooterConstants.SHOOTER_VELOCITY / 2 * Math.PI * ShooterConstants.SHOOTER_LAUNCH_DIAMETER)),
                // I am not sure if this works below - Wesley, If it doesn't use the "new WaitCommand(0.5)," instead
                new InstantCommand(() -> {
                    while(!shooter.shooterAtMaxSpeed) {
                        try {
                            Thread.sleep(20); // each periodic
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }),
                new InstantCommand(()-> shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER))
            )
            ).onFalse(
                new InstantCommand(()->{
                    shooter.deactivateShooterAndFeeder();
                })
            );
        }

        //Just align but don't shoot
        driver.get(PS5Button.TRIANGLE).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()-> setAlignmentPose()),
                new DriveToPose(getDrivetrain(), ()-> alignmentPose)
            )
        );

        //Intake
        if(intake != null){
            driver.get(PS5Button.CROSS).onTrue(
                new IntakeBall(intake, shooter)
            ).onFalse(
                new InstantCommand(()->{
                    intake.setSetpoint(IntakeConstants.STOW_ANGLE);
                    intake.stopFlyWheel();
                })
            );
        }

        //Cancel commands
        driver.get(PS5Button.RB).onTrue(new InstantCommand(()->{
            if(intake != null){
                intake.stopFlyWheel();
                intake.setSetpoint(IntakeConstants.STOW_ANGLE);
            }
            if(shooter != null){
                shooter.stopFeeder();
                shooter.stopShooter();
            }
            if(hood != null){
                hood.setSetpoint(HoodConstants.START_ANGLE);
            }
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(()->null);
            alignmentPose = null;
            CommandScheduler.getInstance().cancelAll();
        }));
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
