package frc.robot.commands.auto_comm;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.gpm.IntakeMovementCommand;
import frc.robot.commands.gpm.RunSpindexer;
import frc.robot.commands.gpm.RunSpindexerWithStop;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;

public class ChoreoPathCommandBuilder {

  private Intake intake;
  private Spindexer spindexer;
  private Turret turret;
  private Hood hood;

  public ChoreoPathCommandBuilder(Intake intake, Spindexer spindexer, Turret turret, Hood hood) {
    this.intake = intake;
    this.spindexer = spindexer;
    this.turret = turret;
    this.hood = hood;

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
              hood.forceHoodDown(true);
            }),
            liberalSwipe.cmd()));

    liberalSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    shallowSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    return routine;

  }

  public AutoRoutine rightLiberal(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("rightLiberal");

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

  public AutoRoutine leftConservative(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("leftConservative");

    AutoTrajectory liberalSwipe = routine.trajectory("conservative", 0);
    AutoTrajectory shallowSwipe = routine.trajectory("conservative", 1);

    routine.active().onTrue(
        Commands.sequence(
            liberalSwipe.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            liberalSwipe.cmd()));

    liberalSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    shallowSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    return routine;

  }

  public AutoRoutine rightConservative(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("rightConservative");

    AutoTrajectory liberalSwipe = routine.trajectory("conservative", 0).mirrorY();
    AutoTrajectory shallowSwipe = routine.trajectory("conservative", 1).mirrorY();

    routine.active().onTrue(
        Commands.sequence(
            liberalSwipe.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            liberalSwipe.cmd()));

    liberalSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    shallowSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    return routine;

  }

  public AutoRoutine leftShallow(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("leftShallow");

    AutoTrajectory shallowSwipe = routine.trajectory("shallowSwipe");

    routine.active().onTrue(
        Commands.sequence(
            shallowSwipe.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    shallowSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    return routine;

  }

  public AutoRoutine rightShallow(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("rightShallow");

    AutoTrajectory shallowSwipe = routine.trajectory("shallowSwipe").mirrorY();

    routine.active().onTrue(
        Commands.sequence(
            shallowSwipe.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    shallowSwipe.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            shallowSwipe.cmd()));

    return routine;

  }

  public AutoRoutine leftSuperShuttling(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("leftSuperShuttling");

    AutoTrajectory shuttlingTrajectory = routine.trajectory("superShuttling");

    routine.active().onTrue(Commands.sequence(
        shuttlingTrajectory.resetOdometry(),
        new InstantCommand(() -> {
          intake.extend();
          intake.spinStart();
        }),
        shuttlingTrajectory.cmd()));

    routine.active().whileTrue(new RunSpindexer(spindexer, turret, hood, intake));

    shuttlingTrajectory.done().onTrue(Commands.sequence(
        new InstantCommand(() -> {
          hood.forceHoodDown(false);
        }),
        new WaitCommand(2.0).raceWith(new IntakeMovementCommand(intake)),
        new InstantCommand(() -> {
          intake.extend();
          intake.spinStart();
          hood.forceHoodDown(true);
        }),
        shuttlingTrajectory.cmd()));

    return routine;

  }

  public AutoRoutine rightSuperShuttling(AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("rightSuperShuttling");

    AutoTrajectory shuttlingTrajectory = routine.trajectory("superShuttling");

    routine.active().onTrue(Commands.sequence(
        shuttlingTrajectory.resetOdometry(),
        new InstantCommand(() -> {
          intake.extend();
          intake.spinStart();
        }),
        shuttlingTrajectory.cmd()));

    routine.active().whileTrue(new RunSpindexer(spindexer, turret, hood, intake));

    shuttlingTrajectory.done().onTrue(Commands.sequence(
        new InstantCommand(() -> {
          hood.forceHoodDown(false);
        }),
        new WaitCommand(2.0).raceWith(new IntakeMovementCommand(intake)),
        new InstantCommand(() -> {
          intake.extend();
          intake.spinStart();
          hood.forceHoodDown(true);
        }),
        shuttlingTrajectory.cmd()));

    return routine;

  }

/**
 * doubleConservativeKousha 
 * @param factory AutoFactory
 * @param right boolean True for right auto, false for left
 * @return AutoRoutine
 */
  public AutoRoutine doubleConservativeKousha(AutoFactory factory, boolean right) {
    AutoRoutine routine = factory.newRoutine(right ? "right" : "left" + "doubleConservativeKousha");

    AutoTrajectory swipe1 = right ? routine.trajectory("doubleConservativeKousha", 0).mirrorY()
        : routine.trajectory("doubleConservativeKousha", 0);
    AutoTrajectory swipe2 = right ? routine.trajectory("doubleConservativeKousha", 1).mirrorY()
        : routine.trajectory("doubleConservativeKousha", 1);
    AutoTrajectory centerSprint = right ? routine.trajectory("doubleConservativeKousha", 2).mirrorY()
        : routine.trajectory("doubleConservativeKousha", 2);

    routine.active().onTrue(
        Commands.sequence(
            swipe1.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            swipe1.cmd()));

    swipe1.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            swipe2.cmd()));

    swipe2.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            centerSprint.cmd()));

    return routine;

  }

/**
 * depotKousha 
 * @param factory AutoFactory
 * @param right boolean True for right auto, false for left
 * @return AutoRoutine
 */
  public AutoRoutine depotKousha(AutoFactory factory, boolean right) {
    AutoRoutine routine = factory.newRoutine(right ? "right" : "left" + "depotKousha");

    AutoTrajectory getDepot = right ? routine.trajectory("depotKousha", 0).mirrorY()
        : routine.trajectory("depotKousha", 0);
    AutoTrajectory shootToTrench = right ? routine.trajectory("depotKousha", 1).mirrorY()
        : routine.trajectory("depotKousha", 1);
    AutoTrajectory centerSprint = right ? routine.trajectory("depotKousha", 2).mirrorY()
        : routine.trajectory("depotKousha", 2);

    routine.active().onTrue(
        Commands.sequence(
            getDepot.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            getDepot.cmd()));

    getDepot.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            Commands.race(
                new RunSpindexer(spindexer, turret, hood, intake),
                new IntakeMovementCommand(intake),
                shootToTrench.cmd())));

    shootToTrench.done()
        .onTrue(Commands.parallel(
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            centerSprint.cmd()));

    return routine;

  }

/**
 * doubleLiberalKousha 
 * @param factory AutoFactory
 * @param right boolean True for right auto, false for left
 * @return AutoRoutine
 */
  public AutoRoutine doubleLiberalKousha(AutoFactory factory, boolean right) {
    AutoRoutine routine = factory.newRoutine(right ? "right" : "left" + "doubleLiberalKousha");

    AutoTrajectory swipe1 = right ? routine.trajectory("doubleLiberalKousha", 0).mirrorY()
        : routine.trajectory("doubleLiberalKousha", 0);
    AutoTrajectory swipe2 = right ? routine.trajectory("doubleLiberalKousha", 1).mirrorY()
        : routine.trajectory("doubleLiberalKousha", 1);
    AutoTrajectory centerSprint = right ? routine.trajectory("doubleLiberalKousha", 2).mirrorY()
        : routine.trajectory("doubleLiberalKousha", 2);

    routine.active().onTrue(
        Commands.sequence(
            swipe1.resetOdometry(),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            swipe1.cmd()));

    swipe1.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            swipe2.cmd()));

    swipe2.done()
        .onTrue(Commands.sequence(
            new InstantCommand(() -> {
              hood.forceHoodDown(false);
            }),
            new RunSpindexerWithStop(spindexer, turret, hood, intake).raceWith(new IntakeMovementCommand(intake)),
            new InstantCommand(() -> {
              intake.extend();
              intake.spinStart();
              hood.forceHoodDown(true);
            }),
            centerSprint.cmd()));

    return routine;

  }

}
