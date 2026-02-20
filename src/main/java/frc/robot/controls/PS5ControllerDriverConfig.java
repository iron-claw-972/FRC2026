package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climb.LinearClimb;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = ()->false;
    private LinearClimb climb;
    private Intake intake;

    private boolean intakeBoolean = true;

    public PS5ControllerDriverConfig(Drivetrain drive, LinearClimb climb, Intake intake) {
        super(drive);
        this.climb = climb;
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

        // Intake
        if (intake != null) {
            driver.get(PS5Button.CROSS).onTrue(new InstantCommand(() -> {
                if (intakeBoolean) {
                    intake.extend();
                    intake.spinStart();
                    intakeBoolean = false;
                } else {
                    intake.intermediateExtend();
                    intake.spinStop();
                    intakeBoolean = true;
                }
            }));

            driver.get(PS5Controller.DPad.DOWN).onTrue(new InstantCommand(() -> {
                intake.retract();
                intakeBoolean = true;
            }));
        }
        
        if (climb != null) {
            driver.get(PS5Button.CIRCLE).onTrue(new InstantCommand(() -> {
                climb.hardstopCalibration();
            })).onFalse(new InstantCommand(() -> {
                climb.stopCalibrating();
            }));
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
