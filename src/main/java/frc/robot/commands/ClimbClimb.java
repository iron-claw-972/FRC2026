package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbClimb extends Command {
    Climb climb;

    public final double CLIMB_SETPOINT = 1.0; //TODO add proper value

    public ClimbClimb(Climb climb){
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setSetpoint(CLIMB_SETPOINT);
    }

    @Override
    public boolean isFinished() {
        return climb.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        climb.setSetpoint(0);
    }

}
