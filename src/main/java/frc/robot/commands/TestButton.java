package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TestButton extends Command{
    int counter;
    
    public void initialize(){
        counter = 0;
        SmartDashboard.putBoolean("eileen's note", true);
    }

    public void execute(){
        counter++;
    }

    public boolean isFinished(){
        return counter > 200;
    }


    public void end(boolean interrupted){
        SmartDashboard.putBoolean("eileen's note", false);
    }
}
