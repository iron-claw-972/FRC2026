package frc.robot.commands.gpm;

import java.util.Optional;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.ShotInterpolation;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.ShooterPhysics;
import frc.robot.util.ShooterPhysics.Constraints;
import frc.robot.util.ShooterPhysics.TurretState;

public class AutoShootCommand extends Command {
    private Turret turret;
    private Drivetrain drivetrain;
    private Hood hood;
    private Shooter shooter;

    //TODO: find maximum interpolation
    private Constraints shooterConstraints = new Constraints(Units.inchesToMeters(80.0), 67676767, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

    private double turretSetpoint;
    private double hoodSetpoint;

    private Pose2d drivepose;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));
    
    private Rotation2d lastTurretAngle;
    private Rotation2d turretAngle;
    private double turretVelocity;

    private double lastHoodAngle;
    private double hoodAngle;
    private double hoodVelocity;

    private TurretState goalState;

    private final double phaseDelay = 0.03;

    public AutoShootCommand(Turret turret, Drivetrain drivetrain, Hood hood, Shooter shooter) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
        drivepose  = drivetrain.getPose();

        goalState = ShooterPhysics.getShotParams(
				new Translation2d(0, 0),
				FieldConstants.getHubTranslation().minus(new Translation3d(drivepose.getTranslation())),
				8.0);
        
        addRequirements(turret, hood);
    }

    public void updateSetpoints(Pose2d drivepose) {

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
            goalState = ShooterPhysics.getShotParams(
				new Translation2d(turretVelocityX, turretVelocityY),
				new Translation3d(target.minus(lookaheadPose.getTranslation())),
				8.0);

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

        // Shortest path
        double error = MathUtil.inputModulus(turretAngle.getDegrees() - Units.radiansToDegrees(turret.getPositionRad()), -180, 180);
        double potentialSetpoint = Units.radiansToDegrees(turret.getPositionRad()) + error;
        // Stay within +/- 200 -- if  shortest path is past 200, we go long way around
        double turretRange = TurretConstants.MAX_ANGLE - TurretConstants.MIN_ANGLE;
        if (potentialSetpoint > turretRange/2) {
            potentialSetpoint -= 360;
        } else if (potentialSetpoint < -turretRange/2) {
            potentialSetpoint += 360;
        }

        turretSetpoint = potentialSetpoint;

        // Hood stuff
        //hoodAngle = ShotInterpolation.hoodAngleMap.get(lookaheadTurretToTargetDistance);
        // Pitch is in radians
        hoodAngle = goalState.pitch();
        hoodSetpoint = MathUtil.clamp(Units.radiansToDegrees(hoodAngle), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.LOOP_TIME);
        lastHoodAngle = hoodAngle;
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
    public void execute() {
        updateDrivePose();
        updateSetpoints(drivepose);
        turret.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(turretSetpoint)), turretVelocity - drivetrain.getAngularRate(2));
        hood.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(hoodSetpoint)), hoodVelocity);
        shooter.setShooter(Units.radiansToRotations(goalState.exitVel() / (ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2)));
    }

    @Override
    public void end(boolean interrupted) {
        // Set the turret to a safe position when the command ends
        turret.setFieldRelativeTarget(new Rotation2d(0.0), 0.0);
        hood.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE)), 0.0);
    }

}