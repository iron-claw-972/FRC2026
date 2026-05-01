package frc.robot.commands.auto_comm;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.gpm.IntakeCommand;
import frc.robot.commands.gpm.IntakeMovementCommand;
import frc.robot.commands.gpm.RunSpindexerWithStop;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class ChoreoPathCommand {

    private Intake intake;
    private Spindexer spindexer;
    private Turret turret;
    private Hood hood;

    public ChoreoPathCommand(Intake intake, Spindexer spindexer, Turret turret, Hood hood) {
        this.intake = intake;
        this.spindexer = spindexer;
        this.turret = turret;
        
    }

  public static Command basicTrajectoryAuto(String pathName, boolean resetOdemetry, AutoFactory factory) {
    Command command = factory.trajectoryCmd(pathName);

    return Commands.sequence(
        resetOdemetry ? new InstantCommand(() -> factory.resetOdometry(pathName)) : new DoNothing(),
        command);
  }

  public AutoRoutine leftLiberal(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("leftLiberal");

    AutoTrajectory liberalSwipe = routine.trajectory("liberal", 0);
    AutoTrajectory shallowSwipe = routine.trajectory("liberal", 1);

    routine.active().onTrue(
        Commands.sequence(
            liberalSwipe.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
            }),
            liberalSwipe.cmd()));

    liberalSwipe.done()
        .onTrue(Commands.sequence(
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            shallowSwipe.cmd()));

    shallowSwipe.done()
        .onTrue(Commands.sequence(
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            shallowSwipe.cmd()));

    return routine;

  }

  public AutoRoutine rightLiberal(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("leftLiberal");

    AutoTrajectory liberalSwipe = routine.trajectory("liberal", 0).mirrorY();
    AutoTrajectory shallowSwipe = routine.trajectory("liberal", 1).mirrorY();

    routine.active().onTrue(
        Commands.sequence(
            liberalSwipe.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
            }),
            liberalSwipe.cmd()));

    liberalSwipe.done()
        .onTrue(Commands.sequence(
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            shallowSwipe.cmd()));

    shallowSwipe.done()
        .onTrue(Commands.sequence(
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            shallowSwipe.cmd()));

    return routine;

  }
}
