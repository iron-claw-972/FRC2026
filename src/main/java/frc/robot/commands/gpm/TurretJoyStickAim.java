package frc.robot.commands.gpm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.turret.Turret;

public class TurretJoyStickAim extends Command{
    private Turret turret;
    private PS5ControllerDriverConfig driver;

    public TurretJoyStickAim(Turret turret){
        this.turret = turret;
    }

    Rotation2d rotation2d = new Rotation2d(driver.getRawSideTranslation(), driver.getRawForwardTranslation());
    double angle = Units.radiansToDegrees(MathUtil.angleModulus(rotation2d.getDegrees()));

    @Override
    public void execute() {
        turret.setSetpoint(angle, 0);
    }

}
