package frc.robot.commands.gpm;

import java.lang.reflect.Field;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.ShotInterpolation;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FieldZone;
import frc.robot.util.ShootingTarget;
import frc.robot.util.Vision.TurretVision;

public class SimpleAutoShoot extends Command {
    private Turret turret;
    private Drivetrain drivetrain;
    private TurretVision turretVision;
    private Shooter shooter;

    private double fieldAngleRad;
    private double turretSetpoint;
    private double adjustedSetpoint;
    private double yawToTagCamera;
    private double yawToTag;

    private boolean turretVisionEnabled = false;
    private boolean SOTM = true;
    private Translation2d drivepose;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));
    private double lastTurretAngle = 0;
    private double lastTurretVelocity = 0;

    public SimpleAutoShoot(Turret turret, Drivetrain drivetrain, Shooter shooter) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        drivepose  = drivetrain.getPose().getTranslation();
        
        addRequirements(turret);
    }

    public void updateTurretSetpoint(Translation2d drivepose) {
        
        //FieldZone currentZone = getZone(drivepose);
        Translation2d target = FieldConstants.getHubTranslation().toTranslation2d();

        double D_y;
        double D_x;
        // TODO: Change time to goal on actual comp bot
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

        // System.out.println("Aligning the turn to degree angle: " + turretSetpoint);
    }

    public void adjustWithTurretCam() {
        if(turretVision.canSeeTag(26) || turretVision.canSeeTag(10)){
            // Adjust turret setpoint based on vision input
            if(Robot.getAlliance() == Alliance.Blue){
                yawToTagCamera = -1 * Units.radiansToDegrees(turretVision.getYawToTagRad(26).get());
            }
            else{
                yawToTagCamera = -1 * Units.radiansToDegrees(turretVision.getYawToTagRad(10).get());
            }
            double error = yawToTagCamera - yawToTag;
            adjustedSetpoint = turretSetpoint + error;
        }
    }

    public void updateYawToTag(){
        // Calculate the yaw offset to the tag
        double D_y = FieldConstants.getHubTranslation().getY() - drivetrain.getPose().getY();
        double D_x = FieldConstants.getHubTranslation().getX() - drivetrain.getPose().getX();
        double angleToTag = Units.radiansToDegrees(Math.atan(D_y / D_x));
        yawToTag = angleToTag - Units.radiansToDegrees(fieldAngleRad);
    }

    @Override
    public void initialize() {
        // Initialize setpoint calculation and set the initial goal for the turret
        updateTurretSetpoint(drivepose);
        // turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), drivetrain.getAngularRate(2));
        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), 0);
    }

    @Override
    public void execute() {
        //shooter.setShooterPower(ShotInterpolation.shooterPowerMap.get(FieldConstants.getHubTranslation().toTranslation2d().getDistance(drivetrain.getPose().getTranslation())));
        // Continuously update setpoints and adjust based on vision if available
        drivepose = drivetrain.getPose().getTranslation();
        updateTurretSetpoint(drivepose);
        updateYawToTag();

        // double turretVelocity =
        // turretAngleFilter.calculate(
        //     new Rotation2d(Units.degreesToRadians(turretSetpoint)).minus(new Rotation2d(Units.degreesToRadians(lastTurretAngle))).getRadians() / Constants.LOOP_TIME);

        double turretAcceleration = ((-drivetrain.getAngularRate(2)) - (lastTurretVelocity)) / Constants.LOOP_TIME;
        
        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), -drivetrain.getAngularRate(2));
        // turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), (-drivetrain.getAngularRate(2)) + turretAcceleration * 0.3);
        //turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), turretVelocity);

        lastTurretAngle = turretSetpoint;
        lastTurretVelocity = -drivetrain.getAngularRate(2);
    }

    @Override
    public void end(boolean interrupted) {
        // Set the turret to a safe position when the command ends
        turret.setFieldRelativeTarget(new Rotation2d(0.0), 0.0);
    }

    public boolean leftSide(Translation2d drivepose) {
        if (drivepose.getY() > (FieldConstants.FIELD_WIDTH / 2)) {
            return true;
        } else {
            return false;
        }
    }

    public FieldZone getZone(Translation2d drivepose) {
        return FieldConstants.getZone(drivepose);
    }
}

