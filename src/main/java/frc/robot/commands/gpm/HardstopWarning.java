package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.turret.Turret;

public class HardstopWarning extends Command {
	private Hood hood;
	private Intake intake;
	private Turret turret;
	private String turretStatus;

	public HardstopWarning(Hood hood, Intake intake, Turret turret) {
		this.hood = hood;
		this.intake = intake;
		this.turret = turret;
		turretStatus = "Unknown";
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void execute() {
		double epsilon = 0.05;
		SmartDashboard.putBoolean("Hood OK", hood.getPositionDeg() >= HoodConstants.MIN_ANGLE - epsilon);
		SmartDashboard.putBoolean("Intake OK", intake.getPosition() >= IntakeConstants.STARTING_POINT - epsilon);

		// if (Math.abs(turret.getPositionRad()) <= epsilon) {
		// 	var encoderPositions = turret.getEncoderPositions();
		// 	if (Math.abs(encoderPositions.getFirst()) <= epsilon && Math.abs(encoderPositions.getSecond()) <= epsilon)
		// 		turretStatus = "Ok";
		// 	else
		// 		turretStatus = "Bad";
		// }

		SmartDashboard.putString("Turret Status", turretStatus);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
