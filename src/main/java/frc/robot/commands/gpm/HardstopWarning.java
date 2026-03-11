package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;

public class HardstopWarning extends Command {
	private Hood hood;
	private Intake intake;

	public HardstopWarning(Hood hood, Intake intake) {
		this.hood = hood;
		this.intake = intake;
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
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
