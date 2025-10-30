package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeReal;

public class IntakeStart extends Command{
    private IntakeReal intake;
    private double setpoint;

    public IntakeStart(IntakeReal intake, double setpoint) {
        this.intake = intake;
        this.setpoint = setpoint;
    }
    @Override
    public void initialize() {
        intake.setSetpoint(setpoint);
    }
    @Override
    public boolean isFinished() {
        return intake.atSetpoint();
    }
}
