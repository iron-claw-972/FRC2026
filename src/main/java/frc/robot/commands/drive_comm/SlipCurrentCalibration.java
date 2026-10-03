package frc.robot.commands.drive_comm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SlipCurrentCalibration extends Command {
    protected final Drivetrain drive;

    public SlipCurrentCalibration(Drivetrain swerve) {
        this.drive = swerve;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        currentModule = 0;
        drive.setStateDeadband(true);

        drive.alignWheels();
        timer.restart();

        // SmartDashboard.putData("Slip Increment", new InstantCommand(() -> {
        //     ++currentModule;
        //     timer.restart();
        // }));
    }

    int currentModule = 0;
    Timer timer = new Timer();
    final double MAX_TIME = 10;
    final double MAX_SPEED = 15;

    @Override
    public void execute() {
        var moduleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
        };

        moduleStates[currentModule] = new SwerveModuleState(MAX_SPEED * timer.get() / MAX_TIME, new Rotation2d());
        drive.setModuleStates(moduleStates, true);

        Logger.recordOutput("Slip/targetV", MAX_SPEED * timer.get() / MAX_TIME);
        Logger.recordOutput("Slip/currentModule", currentModule);

        if (timer.hasElapsed(MAX_TIME)) {
            currentModule += 1;
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return currentModule >= 4;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
                new SwerveModuleState(0, new Rotation2d()),
        }, true);
    }
}