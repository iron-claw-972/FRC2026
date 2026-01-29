package frc.robot.commands.gpm;

import java.lang.reflect.Field;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FieldZone;
import frc.robot.util.ShootingTarget;
import frc.robot.util.Vision.TurretVision;

public class TurretAutoShoot extends Command {
    private Turret turret;
    private Drivetrain drivetrain;
    private TurretVision turretVision;

    private double fieldAngleRad;
    private double turretSetpoint;
    private double adjustedSetpoint;
    private double yawToTagCamera;
    private double yawToTag;

    private boolean turretVisionEnabled = false;
    private boolean SOTM = true;
    private Translation2d drivepose;
    public TurretAutoShoot(Turret turret, Drivetrain drivetrain, TurretVision turretVision) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.turretVision = turretVision;
        drivepose  = drivetrain.getPose().getTranslation();
        
        addRequirements(turret);
    }

    public TurretAutoShoot(Turret turret, Drivetrain drivetrain) {
        this(turret, drivetrain, null);
    }

    public void updateTurretSetpoint(Translation2d drivepose) {
        
        FieldZone currentZone = getZone(drivepose);
        Translation2d target;
        switch (currentZone) {
            case NEUTRAL:
                target = FieldConstants.getAllianceTranslation(leftSide(drivepose)).toTranslation2d();
            case OPPOSITION:
                target = FieldConstants.getAllianceTranslation(leftSide(drivepose)).toTranslation2d();
            case ALLIANCE:
                target = FieldConstants.getAllianceTranslation(leftSide(drivepose)).toTranslation2d(); // For the shooter we will want to check if active but turret should be fine
            default:
                target = FieldConstants.getHubTranslation().toTranslation2d();
            
            // I also made this for if we want to shoot to the opposing teams area (though we would never haha):
            // target = FieldConstants.getOppositionTranslation(leftSide(drivepose)).toTranslation2d();
        }

        double D_y;
        double D_x;
        // TODO: Change time to goal on actual comp bot
        double timeToGoal = 0.67;
        
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
        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), drivetrain.getAngularRate(2));
    }

    @Override
    public void execute() {
        // Continuously update setpoints and adjust based on vision if available
        updateTurretSetpoint(drivepose);
        updateYawToTag();

        if(turretVision != null && turretVisionEnabled && turret.atGoal()){
            adjustWithTurretCam();
            turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(adjustedSetpoint)), -drivetrain.getAngularRate(2));
        } else{
            turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), -drivetrain.getAngularRate(2) * 1.0);
        }
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

