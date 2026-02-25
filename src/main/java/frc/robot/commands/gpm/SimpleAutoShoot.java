package frc.robot.commands.gpm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;

public class SimpleAutoShoot extends Command {
    private Turret turret;
    private Drivetrain drivetrain;
    private Shooter shooter;

    private double fieldAngleRad;
    private double turretSetpoint;

    private boolean SOTM = true;
    private Translation2d drivepose;

    public SimpleAutoShoot(Turret turret, Drivetrain drivetrain, Shooter shooter) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        drivepose  = drivetrain.getPose().getTranslation();
        
        addRequirements(turret);
    }

    public void updateTurretSetpoint(Translation2d drivepose) {
        
        //FieldZone currentZone = getZone(drivepose);
        Translation2d target = FieldConstants.getHubTranslation().toTranslation2d();

        double D_y;
        double D_x;
        double timeToGoal = 0.0;
        
        // If the robot is moving, adjust the target position based on velocity
        if (SOTM) {
            ChassisSpeeds robotRelVel = drivetrain.getChassisSpeeds();
            ChassisSpeeds fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelVel, drivetrain.getYaw());
            double xVel = fieldRelVel.vxMetersPerSecond;
            double yVel = fieldRelVel.vyMetersPerSecond;
            
            D_y = target.getY() - drivepose.getY() - timeToGoal * yVel;
            D_x = target.getX() - drivepose.getX() - timeToGoal * xVel;
        } else {
            D_y = target.getY() - drivepose.getY();
            D_x = target.getX() - drivepose.getX();
        }

        // Calculate the field-relative angle
        fieldAngleRad = Math.atan2(D_y, D_x);

        // Calculate robot heading and adjust for reverse drive
        double robotHeading = MathUtil.angleModulus((drivetrain.getYaw().getRadians() + Math.PI)); // Reverse drive adjustment

        // Calculate turret setpoint (angle relative to robot heading)
        turretSetpoint = MathUtil.inputModulus(Units.radiansToDegrees(fieldAngleRad - robotHeading), -180.0, 180.0);

    }

    @Override
    public void initialize() {
        // Initialize setpoint calculation and set the initial goal for the turret
        updateTurretSetpoint(drivepose);
        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), 0);
    }

    @Override
    public void execute() {
        // Continuously update setpoints and adjust based on vision if available
        drivepose = drivetrain.getPose().getTranslation();
        updateTurretSetpoint(drivepose);
        
        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), -drivetrain.getAngularRate(2));
        shooter.setShooter(ShooterConstants.SHOOTER_VELOCITY);
        
    }

    @Override
    public void end(boolean interrupted) {
        // Set the turret to a safe position when the command ends
        turret.setFieldRelativeTarget(new Rotation2d(0.0), 0.0);
        shooter.setShooter(0.0);
    }

}

