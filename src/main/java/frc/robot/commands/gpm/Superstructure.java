package frc.robot.commands.gpm;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

public class Superstructure extends Command {
    private Turret turret;
    private Drivetrain drivetrain;
    private Hood hood;
    private Shooter shooter;
    private Spindexer spindexer;

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

    private LoggedNetworkNumber phaseDelay = new LoggedNetworkNumber("/Tuning/OPERATOR/Phase Delay", 0.03); //Extrapolation delay due to latency

    private Translation2d target = FieldConstants.HUB_BLUE.toTranslation2d();

    private PhaseManager phaseManager = new PhaseManager();

    private LoggedNetworkNumber hoodOffset = new LoggedNetworkNumber("/Tuning/OPERATOR/Hood Offset", 0.0);

    private LoggedNetworkNumber turretOffset = new LoggedNetworkNumber("/Tuning/OPERATOR/Turret Offet",0.0);

    private double distanceFromTarget = 0.0;
    private LoggedNetworkNumber TOFAdjustment = new LoggedNetworkNumber("/Tuning/OPERATOR/TOF Adjustment", 1.1);

    public Superstructure(Turret turret, Drivetrain drivetrain, Hood hood, Shooter shooter, Spindexer spindexer) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
        this.spindexer = spindexer;
        drivepose  = drivetrain.getPose();

        goalState = ShooterPhysics.getShotParams(
				Translation2d.kZero,
				FieldConstants.getHubTranslation().minus(new Translation3d(drivepose.getTranslation())),
				8.0); // Random initial goalState to prevent it being null
        
        addRequirements(turret, shooter);
    }

    public void updateSetpoints(Pose2d drivepose) {
		Pose2d turretPosition = drivepose.transformBy(
				new Transform2d(TurretConstants.DISTANCE_FROM_ROBOT_CENTER.toTranslation2d(), Rotation2d.kZero));
        
        // If the robot is moving, adjust the target position based on velocity
        ChassisSpeeds robotRelVel = drivetrain.getChassisSpeeds();
        ChassisSpeeds fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelVel, drivetrain.getYaw());

        // Rotational adjustment is not being used, since turret is in center of robot
        double turretVelocityX =
            fieldRelVel.vxMetersPerSecond;
        double turretVelocityY =
            fieldRelVel.vyMetersPerSecond;

        double timeOfFlight;
        Pose2d lookaheadPose;

        Translation2d translationToTarget = target.minus(turretPosition.getTranslation());
        double distanceToTarget = translationToTarget.getNorm();
        
        Translation2d unitToTarget = translationToTarget.div(distanceToTarget);
        
        // positive = moving away, negative = moving toward
        // https://en.wikipedia.org/wiki/Dot_product#Physics
        // dot product of two vectors is same as magnitude(a) * magnitude(b) * cos(theta) if theta is distance between the vecs
        // this means that the dot product is also the magnitude of the sum of the vectors
        double velocityTowardTarget = 
            new Translation2d(turretVelocityX, turretVelocityY).dot(unitToTarget);
        
        Translation3d turret3d = new Translation3d(turretPosition.getX(), turretPosition.getY(), 
            TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getZ());

        // if not aim at hub, aima t ground
        Translation3d target3d = new Translation3d(target.getX(), target.getY(),
            target.equals(FieldConstants.getHubTranslation().toTranslation2d()) ?
            FieldConstants.getHubTranslation().getZ() : 0.0);

        goalState = ShooterPhysics.getShotParams(
            Translation2d.kZero,
            target3d.minus(turret3d),
            2.0);

        // tof = distance / (projectile_speed - velocity_toward_target)
        double projectileSpeedHorizontal = goalState.exitVel() * Math.cos(goalState.pitch());
        double speedDifference = projectileSpeedHorizontal - velocityTowardTarget;
        
        //cant be dividing by zero, or approaching infinity, that would be bad (≧︿≦)
        if (speedDifference <= 0.1) {
            timeOfFlight = goalState.timeOfFlight();
        } else {
            timeOfFlight = distanceToTarget / speedDifference;
        }
        
        timeOfFlight *= TOFAdjustment.get();
        
        double offsetX = turretVelocityX * timeOfFlight;
        double offsetY = turretVelocityY * timeOfFlight;
        lookaheadPose = new Pose2d(
            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
            turretPosition.getRotation());

        turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        if (lastTurretAngle == null) {
            lastTurretAngle = turretAngle;
        }

        // Take the filtered average as the turret's velocity when robot is moving translationally
        turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / Constants.LOOP_TIME);
        
        lastTurretAngle = turretAngle;
        
        if (!Constants.DISABLE_LOGGING) {
            Logger.recordOutput("Turret/Target Pose", target);
            Logger.recordOutput("Lookahead Pose", lookaheadPose);
        }

        // Subtract the rotational angle of the robot from the setpoint
        double adjustedTurretSetpoint = MathUtil.angleModulus(turretAngle.getRadians() - drivepose.getRotation().getRadians());

        // Shortest path
        double error = MathUtil.inputModulus(Units.radiansToDegrees(adjustedTurretSetpoint) - Units.radiansToDegrees(turret.getPositionRad()), -180, 180);
        double potentialSetpoint = Units.radiansToDegrees(turret.getPositionRad()) + error + turretOffset.get();

        // Stay within physical limits -- if shortest path is past max angle, we go long way around
        if (potentialSetpoint > TurretConstants.MAX_ANGLE) {
            potentialSetpoint -= 360;
        } else if (potentialSetpoint < TurretConstants.MIN_ANGLE) {
            potentialSetpoint += 360;
        }

        turretSetpoint = potentialSetpoint;

        // Pitch is in radians
        hoodAngle = goalState.pitch();
        hoodSetpoint = MathUtil.clamp(Units.radiansToDegrees(hoodAngle), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.LOOP_TIME);
        lastHoodAngle = hoodAngle;

        distanceFromTarget = target.getDistance(lookaheadPose.getTranslation());
        Logger.recordOutput("Shooting/distanceToTarget", distanceFromTarget);
    }

    public void updateDrivePose(){
        Pose2d currentPose = drivetrain.getPose();

        drivepose = new Pose2d(
            currentPose.getTranslation(), 
            // Uncomment this if robot is backwards
            currentPose.getRotation()//.plus(new Rotation2d(Math.PI))
        );
        ChassisSpeeds robotRelVel = drivetrain.getChassisSpeeds();

        // Add a phase delay extrapolation component for latency delay
        // drivepose = drivepose.exp(
        //     new Twist2d(
        //         robotRelVel.vxMetersPerSecond * phaseDelay.get(),
        //         robotRelVel.vyMetersPerSecond * phaseDelay.get(),
        //         robotRelVel.omegaRadiansPerSecond * phaseDelay.get()));
    }

    /**
     * Stops and stows all subsystems involved in the command
     */
    public void stowEverything(){
        turret.setFieldRelativeTarget(new Rotation2d(0.0), 0.0);
        hood.setFieldRelativeTarget(Rotation2d.fromDegrees(HoodConstants.MAX_ANGLE), 0.0);
        shooter.setShooter(0.0);
        spindexer.noIndexing = true;
    }

    public void underLadder(){
        spindexer.noIndexing = true;
    }

    // shoot higher
    public void bumpUpHoodOffset() {
        hoodOffset.set(hoodOffset.get() + 1.0); //1 deg
    }

    // shoot lower
    public void bumpDownHoodOffset() {
        hoodOffset.set(hoodOffset.get() - 1.0); //1 deg
    }

    // aim more left
    public void bumpUpTurretOffset() {
        turretOffset.set(turretOffset.get() + 2.5); //2.5 deg
    }

    // aim more right
    public void bumpDownTurretOffset() {
        turretOffset.set(turretOffset.get() - 2.5); //2.5 deg
    }

    @Override
    public void execute() {
        // Phase manager stuff
        phaseManager.update(drivepose, shooter, turret);
        target = phaseManager.getTarget(drivepose);

        updateDrivePose();
        updateSetpoints(drivepose);

        if (phaseManager.isIdle()) {
            underLadder();
        } else {
            if (spindexer.noIndexing) {
                spindexer.noIndexing = false;
            }
            turret.setFieldRelativeTarget(Rotation2d.fromDegrees(turretSetpoint), turretVelocity - drivetrain.getAngularRate(2));

            boolean shuttling = !target.equals(FieldConstants.getHubTranslation().toTranslation2d()); // if we're aiming at the hub, we're not shuttling

            // shuttling will move the hood so its angles very close to max (less arch)
            if (shuttling) {
                hood.setFieldRelativeTarget(Rotation2d.fromDegrees(ShuttleInterpolation.newHoodMap.get(distanceFromTarget)), hoodVelocity);
            } else {
                hood.setFieldRelativeTarget(Rotation2d.fromDegrees(ShotInterpolation.newHoodMap.get(distanceFromTarget)), hoodVelocity);
            }
            
            // if (FieldConstants.underTrench(x, y)) {
            //     System.out.println("Hood forced down");
            // } else {
            //     hood.forceHoodDown(false);
            // }

            // different maps for shuttling vs shooting. Less powerful when shuttling.
            if (shuttling) {
                shooter.setShooter(-ShuttleInterpolation.shooterVelocityMap.get(distanceFromTarget));
            } else {
                shooter.setShooter(-ShotInterpolation.shooterVelocityMap.get(distanceFromTarget));
            }

            if (!Constants.DISABLE_LOGGING) {
            // record when shuttling
            Logger.recordOutput("Shuttling", shuttling);
            // record distance for tuning if needed
            Logger.recordOutput("Distance From Target", distanceFromTarget);
            }
        }

        if (!Constants.DISABLE_LOGGING) {
            Logger.recordOutput("Turret Calculated Setpoint", turretSetpoint);
            Logger.recordOutput("Hood Calculate Setpoint", hoodSetpoint);
            Logger.recordOutput("Shooter Calculate Velocity", goalState.exitVel());
            
            Logger.recordOutput("DistanceToTarget", target.getDistance(drivepose.getTranslation()));
        }

        // for operator
        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putString("Phase Manager State", phaseManager.getCurrentState().toString());
            
        } else {
            phaseDelay.set(0.03);
        }
    }

    @Override
    public void end(boolean interrupted) {
        stowEverything();
    }

}
