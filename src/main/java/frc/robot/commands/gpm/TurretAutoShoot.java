package frc.robot.commands.gpm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.ShotInterpolation;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.FieldZone;

public class TurretAutoShoot extends Command {
    private Turret turret;
    private Drivetrain drivetrain;

    private double turretSetpoint;

    private Pose2d drivepose;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));
    private Rotation2d lastTurretAngle;
    private Rotation2d turretAngle;
    private double turretVelocity;
    private final double phaseDelay = 0.03;

    public TurretAutoShoot(Turret turret, Drivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        drivepose  = drivetrain.getPose();
        
        addRequirements(turret);
    }

    public void updateTurretSetpoint(Pose2d drivepose) {
        
        Translation2d target = FieldConstants.getHubTranslation().toTranslation2d();
        Pose2d turretPosition = drivepose.transformBy(new Transform2d((TurretConstants.DISTANCE_FROM_ROBOT_CENTER), new Rotation2d()));
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());
        
        // If the robot is moving, adjust the target position based on velocity
        ChassisSpeeds robotRelVel = drivetrain.getChassisSpeeds();
        ChassisSpeeds fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelVel, drivetrain.getYaw());

        double turretVelocityX =
            fieldRelVel.vxMetersPerSecond
                + fieldRelVel.omegaRadiansPerSecond
                    * (TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getY() * Math.cos(drivepose.getRotation().getRadians())
                        - TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getX() * Math.sin(drivepose.getRotation().getRadians()));
        double turretVelocityY =
            fieldRelVel.vyMetersPerSecond
                + fieldRelVel.omegaRadiansPerSecond
                    * (TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getX() * Math.cos(drivepose.getRotation().getRadians())
                        - TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getY() * Math.sin(drivepose.getRotation().getRadians()));

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

        // Loop (20) until lookahreadTurretToTargetDistance converges
        for (int i = 0; i < 20; i++) {
            timeOfFlight = ShotInterpolation.timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose =
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }
        turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        if (lastTurretAngle == null) {
            lastTurretAngle = turretAngle;
        }
        turretVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getRadians() / Constants.LOOP_TIME);
        lastTurretAngle = turretAngle;

        // Add 180 since drivetrain is backwards
        turretSetpoint = MathUtil.inputModulus(turretAngle.getDegrees() + 180, -180.0, 180.0);
    }

    public void updateDrivePose(){
        ChassisSpeeds robotRelVel = drivetrain.getChassisSpeeds();
        drivepose = drivetrain.getPose().exp(
            new Twist2d(
                robotRelVel.vxMetersPerSecond * phaseDelay,
                robotRelVel.vyMetersPerSecond * phaseDelay,
                robotRelVel.omegaRadiansPerSecond * phaseDelay));
    }

    @Override
    public void initialize() {
        updateDrivePose();
        updateTurretSetpoint(drivepose);
        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), turretVelocity);
    }

    @Override
    public void execute() {
        updateDrivePose();
        updateTurretSetpoint(drivepose);

        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), turretVelocity);
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

