package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.gpm.AutoShootCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
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
    private Command autoShoot;

    private boolean intakeBoolean = true;

    public PS5ControllerDriverConfig(Drivetrain drive, Shooter shooter, Turret turret, Hood hood, Intake intake, Spindexer spindexer) {
        super(drive);
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
        this.spindexer = spindexer;
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

        // Intake
        if(intake != null){
            driver.get(PS5Button.CROSS).onTrue(new InstantCommand(()->{
                if(intakeBoolean){
                    intake.setSetpoint(IntakeConstants.INTAKE_ANGLE);
                    intake.setFlyWheel();
                    intakeBoolean = false;
                }
                else{
                    intake.setSetpoint(IntakeConstants.STOW_ANGLE);
                    intake.stopFlyWheel();
                }
            }));
        }

        // Auto shoot
        if(turret != null && hood != null){
            driver.get(PS5Button.SQUARE).onTrue(
            new InstantCommand(()->{
                        if (autoShoot != null && autoShoot.isScheduled()){
                            autoShoot.cancel();
                        } else{
                            autoShoot = new AutoShootCommand(turret, getDrivetrain(), hood, shooter, spindexer);
                            CommandScheduler.getInstance().schedule(autoShoot);
                        }
                    })
        );
        }

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
