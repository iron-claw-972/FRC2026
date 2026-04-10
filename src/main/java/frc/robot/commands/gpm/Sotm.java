package frc.robot.commands.gpm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShotInterpolation;
import frc.robot.constants.ShuttleInterpolation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.PhaseManager;
import frc.robot.util.ShooterPhysics;
import frc.robot.util.ShooterPhysics.TurretState;

public class Sotm extends Command {
    private Turret turret;
    private Drivetrain drivetrain;
    private Hood hood;
    private Shooter shooter;
    private Spindexer spindexer;

    private Pose2d drivepose;


    public Sotm(Turret turret, Drivetrain drivetrain, Hood hood, Shooter shooter, Spindexer spindexer) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
        this.spindexer = spindexer;
        drivepose  = drivetrain.getPose();
        
        addRequirements(turret, shooter);
    }

    public void updateSetpoints(Pose2d drivepose) {

    }
    
    public void stowEverything(){
        turret.setFieldRelativeTarget(new Rotation2d(0.0), 0.0);
        hood.setFieldRelativeTarget(Rotation2d.fromDegrees(HoodConstants.MAX_ANGLE), 0.0);
        shooter.setShooter(0.0);
        spindexer.noIndexing = true;
    }
    
    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        stowEverything();
    }

}
