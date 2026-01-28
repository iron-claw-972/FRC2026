package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;

public class DoSysidThings extends SequentialCommandGroup {
	Turret turret;

	public DoSysidThings(Turret turret) {
		this.turret = turret;
		Config conf = new Config();
		SysIdRoutine.Mechanism mech = new SysIdRoutine.Mechanism(turret::setVoltage, this::doLog, turret);
		SysIdRoutine routine = new SysIdRoutine(conf, mech);

		addCommands(
				// turret.goTo(TurretConstants.MIN_ANGLE + 10),
				routine.quasistatic(SysIdRoutine.Direction.kForward).withDeadline(new WaitCommand(4)),
				// turret.goTo(TurretConstants.MAX_ANGLE - 10),
				routine.quasistatic(SysIdRoutine.Direction.kReverse).withDeadline(new WaitCommand(4)),
				// turret.goTo(TurretConstants.MIN_ANGLE + 10),
				routine.dynamic(SysIdRoutine.Direction.kForward).withDeadline(new WaitCommand(2)),
				// turret.goTo(TurretConstants.MAX_ANGLE - 10),
				routine.dynamic(SysIdRoutine.Direction.kReverse).withDeadline(new WaitCommand(2))
				// turret.goTo(0)
				);
	}

	private void doLog(SysIdRoutineLog log) {
		log.motor("turret")
				.voltage(turret.getVolts())
				.angularPosition(turret.getPosition())
				.angularVelocity(turret.getVelocity())
				.angularAcceleration(turret.getAccel());
	}
}
