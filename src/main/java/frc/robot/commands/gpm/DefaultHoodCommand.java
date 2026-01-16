package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.HoodReal;

public class DefaultHoodCommand extends Command{
    private HoodReal hood;
    private Drivetrain drive;

    public DefaultHoodCommand(HoodReal hood, Drivetrain drive){
        this.hood = hood;
        this.drive = drive;

        addRequirements(hood);
    }

    @Override
    public void execute(){
        hood.aimToTarget(drive.getPose());
    }
}
