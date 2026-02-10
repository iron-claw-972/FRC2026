package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.gpm.SimpleAutoShoot;
import frc.robot.commands.gpm.TurretAutoShoot;
import frc.robot.commands.gpm.TurretJoyStickAim;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = ()->false;
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Spindexer spindexer;

    private Pose2d alignmentPose = null;
    private Command turretAutoShoot;
    private Command simpleTurretAutoShoot;
    private TurretJoyStickAim turretJoyStickAim;

    public PS5ControllerDriverConfig(Drivetrain drive, Shooter shooter, Turret turret, Hood hood) {
        super(drive);
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
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

        if(intake != null){
            
        }

        // driver.get(PS5Button.LB).onTrue(
        //     new SequentialCommandGroup(
        //         new InstantCommand(()-> shooter.setShooter(-ShooterConstants.SHOOTER_VELOCITY)),
        //         new WaitCommand(0.8),
        //         new InstantCommand(()-> shooter.setFeeder(ShooterConstants.FEEDER_RUN_POWER))
        //     )
        //     ).onFalse(
        //         new InstantCommand(() -> {
        //                 shooter.setFeeder(0);
        //                 shooter.setShooter(0);
        //             }));
        //driver.get(PS5Button.TRIANGLE).onTrue(new InstantCommand(() -> shooter.setShooter(ShooterConstants.SHOOTER_VELOCITY))).onFalse(new InstantCommand(() -> shooter.setShooter(0)));
        
        // driver.get(PS5Button.SQUARE).onTrue(
        //     new InstantCommand(()->{
        //                 if (simpleTurretAutoShoot != null && simpleTurretAutoShoot.isScheduled()){
        //                     simpleTurretAutoShoot.cancel();
        //                 } else{
        //                     simpleTurretAutoShoot = new SimpleAutoShoot(turret, getDrivetrain(), shooter);
        //                     CommandScheduler.getInstance().schedule(simpleTurretAutoShoot);
        //                 }
        //             })
        // );


        driver.get(PS5Button.SQUARE).onTrue(new InstantCommand(() -> hood.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE)), 0)));
        driver.get(PS5Button.TRIANGLE).onTrue(new InstantCommand(() -> hood.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MIN_ANGLE)), 0)));
        driver.get(PS5Button.LB).onTrue(new InstantCommand(() -> hood.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians((HoodConstants.MAX_ANGLE + HoodConstants.MIN_ANGLE) / 2)), 0)));


        // driver.get(PS5Button.CIRCLE).onTrue(
        //     new InstantCommand(()->{
        //         turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(180)), 0);
        //     })
        // ).onFalse(
        //     new InstantCommand(()->{
        //         turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(-170)), 0);
        //     })
        // );

        // driver.get(PS5Button.CROSS).onTrue(
        //     new InstantCommand(()->{
        //         if(turretJoyStickAim == null || !turretJoyStickAim.isScheduled()){
        //             turretJoyStickAim = new TurretJoyStickAim(turret, this);
        //             turretJoyStickAim.schedule();
        //         }
        //     })
        // ).onFalse(
        //     new InstantCommand(()->{
        //         if(turretJoyStickAim.isScheduled()){
        //             turretJoyStickAim.cancel();
        //         }
        //     })
        // );
        
    }
    
    public void setAlignmentPose(){
        Translation2d drivepose = getDrivetrain().getPose().getTranslation();
        // uses tag #??
        int tagNumber = 17;
        Translation2d tagpose = FieldConstants.field.getTagPose(tagNumber).get().toPose2d().getTranslation();
        double YDifference = tagpose.getY() - drivepose.getY();
        double XDifference = tagpose.getX() - drivepose.getX();
        double angle = Math.atan(YDifference/XDifference);
        alignmentPose = new Pose2d(drivepose.getX(), drivepose.getY(), new Rotation2d(angle));
        System.out.println("Alignment angle: " + Units.radiansToDegrees(angle));
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
