package frc.robot.commands.misc;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.SysId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class TurretSysIDCommand extends SequentialCommandGroup {

  private Config config = new Config();
  private SysId sysId;

  public TurretSysIDCommand(Turret turret) {
    config = new Config(
        Units.Volts.of(0.2).per(Units.Seconds),
        Units.Volts.of(1),
        Units.Seconds.of(5),
        (state) -> Logger.recordOutput("SysIdTestState", state.toString()));
    sysId = new SysId(
        "Drivetrain",
        x -> {
          turret.setVoltage(x);
        },
        turret,
        config);
    addCommands(
        //set turret to min pos before running
        new InstantCommand(() -> turret.setBegin()),
        new WaitCommand(0.5),
        sysId.runQuasisStatic(Direction.kForward),
        new WaitCommand(0.5),
        sysId.runQuasisStatic(Direction.kReverse),
        new WaitCommand(0.5),
        sysId.runDynamic(Direction.kForward),
        new WaitCommand(0.5),
        sysId.runDynamic(Direction.kReverse));
  }

}
