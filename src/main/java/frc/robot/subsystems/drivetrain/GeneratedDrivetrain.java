package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Phoenix 6 drivetrain wrapper.
 *
 * CTRE does hardware construction and odometry fyi
 */
public class GeneratedDrivetrain extends TunerConstants.TunerSwerveDrivetrain implements Subsystem {
    private static final double SIM_LOOP_PERIOD = 0.005;

    protected Notifier simNotifier;
    private double lastSimTime;

    public GeneratedDrivetrain(
            com.ctre.phoenix6.swerve.SwerveDrivetrainConstants drivetrainConstants,
            com.ctre.phoenix6.swerve.SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            double now = Utils.getCurrentTimeSeconds();
            double dt = now - lastSimTime;
            lastSimTime = now;
            updateSimState(dt, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }
}
