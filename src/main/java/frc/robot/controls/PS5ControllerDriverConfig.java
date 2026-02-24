package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.gpm.AutoShootCommand;
import frc.robot.commands.gpm.ClimbDriveCommand;
import frc.robot.commands.gpm.ReverseMotors;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Climb.LinearClimb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.DPad;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = () -> false;
    private boolean intakeBoolean = true;
    private Command autoShoot = null;
    private Command reverseMotors = null;
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Spindexer spindexer;
    private LinearClimb climb;

    public PS5ControllerDriverConfig(
            Drivetrain drive,
            Shooter shooter,
            Turret turret,
            Hood hood,
            Intake intake,
            Spindexer spindexer,
            LinearClimb climb) {
        super(drive);
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
        this.spindexer = spindexer;
        this.climb = climb;
    }

    public void configureControls() {
        // Reset the yaw. Mainly useful for testing/driver practice
        driver.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

        // Cancel commands
        driver.get(PS5Button.RB).onTrue(new InstantCommand(() -> {
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(() -> null);
            CommandScheduler.getInstance().cancelAll();
        }));

        // Align wheels
        driver.get(PS5Button.MUTE).onTrue(new FunctionalCommand(
                () -> getDrivetrain().setStateDeadband(false),
                getDrivetrain()::alignWheels,
                interrupted -> getDrivetrain().setStateDeadband(true),
                () -> false, getDrivetrain()).withTimeout(2));

        // Trench align
        driver.get(DPad.LEFT).onTrue(new InstantCommand(() -> getDrivetrain().setTrenchAlign(true)))
                .onFalse(new InstantCommand(() -> getDrivetrain().setTrenchAlign(false)));

        // Trench assist
        driver.get(DPad.RIGHT).onTrue(new InstantCommand(() -> getDrivetrain().setTrenchAssist(true)))
                .onFalse(new InstantCommand(() -> getDrivetrain().setTrenchAssist(false)));

        // Reverse motors
        if (intake != null && spindexer != null && shooter != null) {
            driver.get(PS5Button.CIRCLE).onTrue(new InstantCommand(() -> {
                reverseMotors = new ReverseMotors(intake, spindexer);
                reverseMotors.schedule();
            })).onFalse(new InstantCommand(() -> {
                if (reverseMotors != null) {
                    reverseMotors.cancel();
                }
            }));
        }

        // Intake
        if (intake != null) {
            // Toggle intake
            driver.get(PS5Button.RIGHT_TRIGGER).onTrue(new InstantCommand(() -> {
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

            // Retract if hold for 3 seconds
            driver.get(PS5Button.RIGHT_TRIGGER).debounce(3.0).onTrue(new InstantCommand(() -> {
                intake.retract();
                intakeBoolean = true;
            }));

            // Calibration
            driver.get(PS5Button.PS).onTrue(new InstantCommand(() -> {
                intake.calibrate();
            })).onFalse(new InstantCommand(() -> {
                intake.stopCalibrating();
            }));
        }

        // Spindexer
        if (spindexer != null) {
            // Will only run if we are not calling default shoot command
            driver.get(PS5Button.LEFT_TRIGGER).onTrue(new InstantCommand(() -> spindexer.maxSpindexer()))
                    .onFalse(new InstantCommand(() -> spindexer.stopSpindexer()));
        }

        // Auto shoot
        if (turret != null && hood != null && shooter != null) {
            driver.get(PS5Button.SQUARE).onTrue(
                    new InstantCommand(() -> {
                        if (autoShoot != null && autoShoot.isScheduled()) {
                            autoShoot.cancel();
                        } else {
                            autoShoot = new AutoShootCommand(turret, getDrivetrain(), hood, shooter, spindexer);
                            CommandScheduler.getInstance().schedule(autoShoot);
                        }
                    }));
        }

        // Climb
        if (climb != null) {
            // Calibration
            driver.get(PS5Button.PS).onTrue(new InstantCommand(() -> {
                climb.hardstopCalibration();
            })).onFalse(new InstantCommand(() -> {
                climb.stopCalibrating();
            }));

            // Climb retract
            driver.get(PS5Button.CROSS).onTrue(new InstantCommand(() -> {
                climb.retract();
            }));

            // Drive to climb position and rumble
            driver.get(PS5Button.TRIANGLE).onTrue(new SequentialCommandGroup(
                    new ClimbDriveCommand(climb, getDrivetrain()),
                    new InstantCommand(() -> this.startRumble()),
                    new WaitCommand(1),
                    new InstantCommand(() -> this.endRumble())));
        }

        // Hood
        if (hood != null) {
            // Calibration
            driver.get(PS5Button.PS).onTrue(new InstantCommand(() -> {
                hood.calibrate();
            })).onFalse(new InstantCommand(() -> {
                hood.stopCalibrating();
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

    public void startRumble() {
        driver.rumbleOn();
    }

    public void endRumble() {
        driver.rumbleOff();
    }
}
