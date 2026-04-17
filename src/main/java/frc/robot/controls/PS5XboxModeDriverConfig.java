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
import frc.robot.commands.gpm.ReverseMotors;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/**
 * Driver config for PS5 controllers using Xbox 360 emulation mode.
 * This lets SCUF and other PS5 controllers work with WPILib rumble.
 * 
 * Setup:
 * - download DSX (https://dualsensex.com/download/)
 * - install ViGEmBus driver (if app doesn't auto prompt)
 * - in dsx, set "controller emulation" to Xbox 360
 * - ensure rumble is enabled in dsx settings
 * - once code is depoloyed, change controller to "Xbox 360" in driverstation
 */

public class PS5XboxModeDriverConfig extends BaseDriverConfig {
    private final GameController controller = new GameController(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = () -> false;
    private boolean intakeBoolean = true;
    private Command autoShoot = null;
    private Command reverseMotors = null;
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Spindexer spindexer;

    // PS5 button aliases
    // private final Button CROSS = Button.A;
    private final Button CIRCLE = Button.B;
    private final Button SQUARE = Button.X;
    // private final Button TRIANGLE = Button.Y;
    // private final Button LB = Button.LB;
    private final Button RB = Button.RB;
    private final Button CREATE = Button.BACK;
    // private final Button OPTIONS = Button.START;
    private final Button LEFT_JOY = Button.LEFT_JOY;
    private final Button RIGHT_JOY = Button.RIGHT_JOY;

    // PS5 trigger buttons
    private final BooleanSupplier LEFT_TRIGGER_BUTTON = controller.LEFT_TRIGGER_BUTTON;
    private final BooleanSupplier RIGHT_TRIGGER_BUTTON = controller.RIGHT_TRIGGER_BUTTON;

    // PS5 axis aliases
    private final Axis LEFT_X = Axis.LEFT_X;
    private final Axis LEFT_Y = Axis.LEFT_Y;
    private final Axis RIGHT_X = Axis.RIGHT_X;
    private final Axis RIGHT_Y = Axis.RIGHT_Y;
    // private final Axis LEFT_TRIGGER = Axis.LEFT_TRIGGER;
    // private final Axis RIGHT_TRIGGER = Axis.RIGHT_TRIGGER;

    public PS5XboxModeDriverConfig(
            Drivetrain drive,
            Shooter shooter,
            Turret turret,
            Hood hood,
            Intake intake,
            Spindexer spindexer) {
        super(drive);
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        this.intake = intake;
        this.spindexer = spindexer;
    }

    public void configureControls() {
        // Reset the yaw. Mainly useful for testing/driver practice
        controller.get(CREATE).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

        // Cancel commands
        controller.get(RB).onTrue(new InstantCommand(() -> {
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(() -> null);
            CommandScheduler.getInstance().cancelAll();
        }));

        // Align wheels
        controller.get(DPad.RIGHT).onTrue(new FunctionalCommand(
                () -> getDrivetrain().setStateDeadband(false),
                getDrivetrain()::alignWheels,
                interrupted -> getDrivetrain().setStateDeadband(true),
                () -> false, getDrivetrain()).withTimeout(2));

        // Trench align
        controller.get(DPad.LEFT).onTrue(new InstantCommand(() -> {
            getDrivetrain().setTrenchAssist(true);
            getDrivetrain().setTrenchAlign(true);
        }))
                .onFalse(new InstantCommand(() -> {
                    getDrivetrain().setTrenchAssist(false);
                    getDrivetrain().setTrenchAlign(false);
                }));

        // Reverse motors
        if (intake != null && spindexer != null && shooter != null) {
            controller.get(CIRCLE).onTrue(new InstantCommand(() -> {
                reverseMotors = new ReverseMotors(intake);
                CommandScheduler.getInstance().schedule(reverseMotors);
            })).onFalse(new InstantCommand(() -> {
                if (reverseMotors != null) {
                    reverseMotors.cancel();
                }
            }));
        }

        // Intake
        if (intake != null) {
            // Toggle intake
            controller.get(RIGHT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> {
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
            controller.get(RIGHT_TRIGGER_BUTTON).debounce(3.0).onTrue(new InstantCommand(() -> {
                intake.retract();
                intakeBoolean = true;
            }));

            // Calibration
            controller.get(LEFT_JOY).onTrue(new InstantCommand(() -> {
                intake.calibrate();
            })).onFalse(new InstantCommand(() -> {
                intake.stopCalibrating();
            }));
        }

        // Spindexer
        if (spindexer != null) {
            // Will only run if we are not calling default shoot command
            controller.get(LEFT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> spindexer.maxSpindexer()))
                    .onFalse(new InstantCommand(() -> spindexer.stopSpindexer()));
        }

        // Auto shoot
        if (turret != null && hood != null && shooter != null) {
            controller.get(SQUARE).onTrue(
                    new InstantCommand(() -> {
                        if (autoShoot != null && autoShoot.isScheduled()) {
                            autoShoot.cancel();
                        } else {
                            autoShoot = new AutoShootCommand(turret, getDrivetrain(), hood, shooter, spindexer);
                            CommandScheduler.getInstance().schedule(autoShoot);
                        }
                    }));
        }

        // Hood
        if (hood != null) {
            controller.get(LEFT_JOY).onTrue(new InstantCommand(() -> {
                hood.calibrate();
            })).onFalse(new InstantCommand(() -> {
                hood.stopCalibrating();
            }));
        }

        // Rumble test
        controller.get(RIGHT_JOY).onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> controller.setRumble(GameController.RumbleStatus.RUMBLE_ON)),
                new WaitCommand(0.5),
                new InstantCommand(() -> controller.setRumble(GameController.RumbleStatus.RUMBLE_OFF))));
    }

    @Override
    public double getRawSideTranslation() {
        return controller.get(LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return controller.get(LEFT_Y);
    }

    @Override
    public double getRawRotation() {
        return controller.get(RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(controller.get(RIGHT_X), -controller.get(RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return Math.hypot(controller.get(RIGHT_X), controller.get(RIGHT_Y));
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
        controller.setRumble(GameController.RumbleStatus.RUMBLE_ON);
    }

    public void endRumble() {
        controller.setRumble(GameController.RumbleStatus.RUMBLE_OFF);
    }
}
