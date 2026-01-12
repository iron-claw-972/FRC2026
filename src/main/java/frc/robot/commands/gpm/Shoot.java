package frc.robot.commands.gpm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.drive_comm.DefaultDriveCommand;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.Shooter.ShooterReal;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.HoodReal;
import frc.robot.util.ShooterPhysics.TurretState;
import frc.robot.util.Vision.DetectedObject;
import frc.robot.util.Vision.DetectedObject.ObjectType;

public class Shoot extends DefaultDriveCommand {
    private static DetectedObject cachedObject;

    HoodReal hood;
    ShooterReal shooter;
    TurretState target_state;

    boolean atTarget;

    public Shoot(Drivetrain drive, BaseDriverConfig driver, HoodReal hood, ShooterReal shooter) {
        super(drive, driver);

        this.hood = hood;
        this.shooter = shooter;

    }

    @Override
    public void initialize() {
        super.initialize();
        
    }

    @Override
    protected void drive(ChassisSpeeds speeds) {

        if (!VisionConstants.OBJECT_DETECTION_ENABLED) {
            super.drive(speeds);
            return;
        }


        // System.out.println("curangle " +
        // swerve.getPose().getRotation().getDegrees());
        // System.out.println("objangle " + object.getAngle());
        // swerve.driveHeading(
        //         speeds.vxMetersPerSecond,
        //         speeds.vyMetersPerSecond,
        //         MathUtil.angleModulus(object.getAngle() + Math.PI / 2),
        //         true);
    }
}
