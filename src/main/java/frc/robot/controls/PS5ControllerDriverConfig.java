package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    public final PS5Controller controller = new PS5Controller(Constants.DRIVER_JOY);
    private final BooleanSupplier slowModeSupplier = () -> false;
    private boolean intakeBoolean = true;
    private Command autoShoot = null;
    private Shooter shooter;
    private Turret turret;

    public PS5ControllerDriverConfig(
        Drivetrain drive, 
        Shooter shooter, 
        Turret turret
    ){
        super(drive);
        this.shooter = shooter;
        this.turret = turret;
    }

    public void configureControls() {
        // Reset the yaw. Mainly useful for testing/driver practice
        controller.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
                new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI))));

        // Cancel commands
        controller.get(PS5Button.RIGHT_TRIGGER).onTrue(new InstantCommand(() -> {
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(() -> null);
            CommandScheduler.getInstance().cancelAll();
        }));

        // Align wheels
        controller.get(PS5Button.MUTE).onTrue(new FunctionalCommand(
                () -> getDrivetrain().setStateDeadband(false),
                getDrivetrain()::alignWheels,
                interrupted -> getDrivetrain().setStateDeadband(true),
                () -> false, getDrivetrain()).withTimeout(2));

        controller.get(PS5Button.CROSS).onTrue(new InstantCommand(() -> getDrivetrain().setTrenchAlign(true)))
            .onFalse(new InstantCommand(() -> getDrivetrain().setTrenchAlign(false)));

        controller.get(PS5Button.CIRCLE).onTrue(new InstantCommand(() -> getDrivetrain().setTrenchAssist(true)))
            .onFalse(new InstantCommand(() -> getDrivetrain().setTrenchAssist(false)));




        // // Intake
        // if (intake != null) {
        //     driver.get(PS5Button.CROSS).onTrue(new InstantCommand(() -> {
        //         if (intakeBoolean) {
        //             intake.extend();
        //             intake.spinStart();
        //             intakeBoolean = false;
        //         } else {
        //             intake.intermediateExtend();
        //             intake.spinStop();
        //             intakeBoolean = true;
        //         }
        //     }));

        //     // Retract if hold for 3 seconds
        //     driver.get(PS5Button.CROSS).debounce(3.0).onTrue(new InstantCommand(()->{
        //         intake.retract();
        //         intakeBoolean = true;
        //     }));

        //     // Calibration
        //     driver.get(PS5Button.OPTIONS).onTrue(new InstantCommand(()->{
        //         intake.calibrate();
        //     })).onFalse(new InstantCommand(()->{
        //         intake.stopCalibrating();
        //     }));
        // }

        // // Spindexer
        // if (spindexer != null){
        //     // Will only run if we are not calling default shoot command
        //     driver.get(PS5Button.LB).onTrue(new InstantCommand(()-> spindexer.maxSpindexer()))
        //     .onFalse(new InstantCommand(()-> spindexer.stopSpindexer()));
        // }

        // // Auto shoot
        // if (turret != null) {
        //     driver.get(PS5Button.SQUARE).onTrue(
        //             new InstantCommand(() -> {
        //                 if (autoShoot != null && autoShoot.isScheduled()) {
        //                     autoShoot.cancel();
        //                 } else {
        //                     autoShoot = new AutoShootCommand(turret, getDrivetrain(), hood, shooter, spindexer);
        //                     CommandScheduler.getInstance().schedule(autoShoot);
        //                 }
        //             }));
        // }

        // if (climb != null) {
        //     driver.get(PS5Button.CIRCLE).onTrue(new InstantCommand(() -> {
        //         climb.hardstopCalibration();
        //     })).onFalse(new InstantCommand(() -> {
        //         climb.stopCalibrating();
        //     }));
        // }
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

    public void startRumble() {
        controller.rumbleOn();
    }

    public void endRumble() {
        controller.rumbleOff();
    }
}
