package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;

public class TurretJoyStickAim extends Command{
    private Turret turret;

    public TurretJoyStickAim(Turret turret){
        this.turret = turret;
    }

    Rotation2d desiredAngle = new Rotation2d(-driver.getRawSideTranslation(), driver.getRawForwardTranslation()); //this feels better

}
