package frc.robot.commands.gpm;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.subsystems.turret.Turret;

public class DoSysidThings extends SequentialCommandGroup {
    Turret turret;

    // TODO: CHECK HARDSTOPS
    public DoSysidThings(Turret turret) {
        this.turret = turret;
        Config conf = new Config();
        SysIdRoutine.Mechanism mech = new SysIdRoutine.Mechanism(turret::setVoltage, this::doLog, turret);
        SysIdRoutine routine = new SysIdRoutine(conf, mech);

        addCommands(
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
        )
    }

    private void doLog(SysIdRoutineLog log) {
        log.motor("turret")
            .voltage(turret.getVolts())
            .angularPosition(turret.getPosition())
            .angularVelocity(turret.getVelocity())
            .angularAcceleration(turret.getAccel());
    }
}
