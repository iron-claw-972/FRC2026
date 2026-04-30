package frc.robot.commands.auto_comm;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ChoreoPathCommand extends SequentialCommandGroup {
  public ChoreoPathCommand(String pathName, boolean resetOdemetry, AutoFactory factory) {
    var command = factory.trajectoryCmd(pathName);

    addCommands(
        resetOdemetry ? new InstantCommand(() -> factory.resetOdometry(pathName)) : new DoNothing(),
        command);
  }
}
