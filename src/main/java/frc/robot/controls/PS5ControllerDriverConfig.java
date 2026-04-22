package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.gpm.IntakeMovementCommand;
import frc.robot.commands.gpm.ReverseMotors;
import frc.robot.commands.gpm.RunSpindexer;
import frc.robot.commands.gpm.RunSpindexerWithStop;
import frc.robot.commands.gpm.Superstructure;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.DPad;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private final PS5Controller controller = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = () -> false;
    private boolean intakeBoolean = true;
    private Command autoShoot = null;
    private Shooter shooter;
    private Turret turret;
    private Hood hood;
    private Intake intake;
    private Spindexer spindexer;

    public PS5ControllerDriverConfig(
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
        controller.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

        // Cancel commands
        controller.get(PS5Button.RB).onTrue(new InstantCommand(() -> {
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(() -> null);
            CommandScheduler.getInstance().cancelAll();
        }));

        // Reverse motors
        if (intake != null && spindexer != null) {
            controller.get(PS5Button.LB).whileTrue(new ReverseMotors(intake));
        }

        // Intake
        if (intake != null) {
            // Toggle intake
            controller.get(PS5Button.RIGHT_TRIGGER).onTrue(new InstantCommand(() -> {
                if (intakeBoolean) {
                    intake.extend();
                    intake.spinStart();
                    intakeBoolean = false;
                } else {
                    intake.intermediateExtend();
                    intake.spinStop();
                    intakeBoolean = true;
                }
            }, intake));

            // Retract if hold for 2 seconds
            controller.get(PS5Button.RIGHT_TRIGGER).debounce(2.0).onTrue(new InstantCommand(() -> {
                intake.retract();
                intakeBoolean = true;
                intake.spinStop();
            }, intake));

            // Make the intake go in and out while shooting
            controller.get(DPad.UP).whileTrue(new IntakeMovementCommand(intake)
                .alongWith(new InstantCommand(()-> intakeBoolean = true)));

            // Calibration: you can now calibrate easily using this button
            if (hood != null && intake != null) {
                controller.get(PS5Button.PS).onTrue(new InstantCommand(() -> {
                    intake.calibrate();
                    hood.calibrate();
                }, intake, hood)).onFalse(new InstantCommand(() -> {
                    intake.stopCalibrating();
                    hood.stopCalibrating();
                }, intake, hood));
            }

            // Stop intake roller
            controller.get(DPad.DOWN).onTrue(new InstantCommand(()->{
                if(intakeBoolean){
                    intake.spinStart();
                    intakeBoolean = false;
                } else{
                    intake.spinStop();
                    intakeBoolean = true;
                }
            }));
        }

        // Spindexer
        if (spindexer != null && turret != null && hood != null && intake != null) {

            // Toggle spindexer
            controller.get(PS5Button.LEFT_TRIGGER).toggleOnTrue(
                new RunSpindexer(spindexer, turret, hood, intake)
            );
        }

        // Auto shoot
        if (turret != null && hood != null && shooter != null && spindexer != null) {
            autoShoot = new Superstructure(turret, getDrivetrain(), hood, shooter, spindexer);
            controller.get(PS5Button.SQUARE).toggleOnTrue(autoShoot);
        }

    
        // Hood
        if (hood != null) {
            // Set the hood down -- for safety measures under trench
            controller.get(DPad.LEFT).onTrue(new InstantCommand(()->{
                hood.forceHoodDown(true);
            }, hood)).onFalse(new InstantCommand(()->{
                hood.forceHoodDown(false);
            }));
        }
    }

    @Override
    public double getRawSideTranslation() {
        return controller.get(PS5Axis.LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return controller.get(PS5Axis.LEFT_Y);
    }

    @Override
    public double getRawRotation() {
        return controller.get(PS5Axis.RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(controller.get(PS5Axis.RIGHT_X), -controller.get(PS5Axis.RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return Math.hypot(controller.get(PS5Axis.RIGHT_X), controller.get(PS5Axis.RIGHT_Y));
    }

    @Override
    public boolean getIsSlowMode() {
        return slowModeSupplier.getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return false;
    }
}
