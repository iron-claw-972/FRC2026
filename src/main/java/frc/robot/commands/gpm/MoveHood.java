package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodReal;

public class MoveHood extends Command{
    private HoodReal hood;
    private double setpoint;

    public MoveHood(HoodReal hood, double setpoint){
        this.hood = hood;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        hood.setSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        return hood.atSetpoint();
    }
}
