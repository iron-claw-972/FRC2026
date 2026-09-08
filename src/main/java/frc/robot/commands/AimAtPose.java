package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;

public class AimAtPose extends Command {

    private final Hood hood;
    private final Turret turret;
    private final Translation2d target;

    public AimAtPose(Hood hood, Turret turret, Translation2d target) {
        this.hood = hood;
        this.turret = turret;
        this.target = target;

        addRequirements(hood, turret);
    }

    @Override
    public void execute() {
        double turretAngle = Math.atan2(
            target.getY(),
            target.getX()
        );

        turret.setFieldRelativeTarget(
            new Rotation2d(turretAngle),
            0.0
        );

        hood.setFieldRelativeTarget(
            Rotation2d.fromDegrees(0),
            0.0
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}