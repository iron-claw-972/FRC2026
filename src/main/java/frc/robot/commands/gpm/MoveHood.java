package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;

public class MoveHood extends Command{
    private Hood hood;
    private double setpoint;

    public MoveHood(Hood hood, double setpoint){
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
