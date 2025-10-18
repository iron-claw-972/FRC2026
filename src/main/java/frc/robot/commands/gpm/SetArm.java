package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmComp;

public class SetArm extends Command{
    private ArmComp arm;
    private double setpoint;

    public SetArm(ArmComp arm, double setpoint){
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        if (arm != null){
            arm.setSetpoint(setpoint);
        }
    }

    @Override
    public boolean isFinished(){
        return arm == null || arm.atSetpoint();
    }
    
    
}
