package frc.robot.commands.gpm;

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
import frc.robot.constants.ShotInterpolation;
import frc.robot.constants.ShuttleInterpolation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.PhaseManager;
import frc.robot.util.ShooterPhysics;
import frc.robot.util.ShooterPhysics.TurretState;

public class LockedShoot extends Command {
    private Turret turret;
    private Drivetrain drive;
    private Hood hood;
    private Shooter shooter;

    private double turretSetpoint;
    private double hoodSetpoint;

    private Pose2d drivepose;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_TIME));
    
    private Rotation2d lastTurretAngle;
    private Rotation2d targetAngle;
    private double turretVelocity;

    private double lastHoodAngle;
    private double hoodAngle;
    private double hoodVelocity;

    private TurretState goalState;

    private double phaseDelay = 0.03; // Extrapolation delay due to latency

    private Translation2d target = FieldConstants.HUB_BLUE.toTranslation2d();

    private PhaseManager phaseManager = new PhaseManager();

    private double hoodOffset = 0.0;

    private double turretOffset = 0.0;

    private double distanceFromTarget = 0.0;

    private double TOFAdjustment = 0.85;

    public LockedShoot(Turret turret, Drivetrain drivetrain, Hood hood, Shooter shooter) {
        this.turret = turret;
        this.drive = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
        drivepose = drivetrain.getPose();

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
        ChassisSpeeds robotRelVel = drive.getChassisSpeeds();
        ChassisSpeeds fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelVel, drive.getYaw());

        // Rotational adjustment is not being used, since turret is in center of robot
        double turretVelocityX =
            fieldRelVel.vxMetersPerSecond;
        double turretVelocityY =
            fieldRelVel.vyMetersPerSecond;

        double timeOfFlight = 0;
        Pose2d lookaheadPose = turretPosition;

        /*
         * Loop (max 20) until lookaheadPose converges BECAUSE -->
         * If you're 8m away (t = 1.0s) and moving at 2m/s towards target, you calculate for 6m (t = 0.8s)
         * At 6m, we run assuming t = 0.8 but then the 6m isn't correct since it was derived using t = 1.0s
         * So we make a bunch of guesses until it converges
         * Early exit when change < 1mm to avoid unnecessary iterations
         */
        for (int i = 0; i < 20; i++) {
            Translation3d lookahead3d = new Translation3d(lookaheadPose.getX(), lookaheadPose.getY(), TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getZ());
            
            Translation3d target3d = new Translation3d(target.getX(), target.getY(),
                target.equals(FieldConstants.getHubTranslation().toTranslation2d()) ?
                FieldConstants.getHubTranslation().getZ() : 0.0); // Height of 0 if it's not the hub

            goalState = ShooterPhysics.getShotParams(
            Translation2d.kZero,
            target3d.minus(lookahead3d),
            2.0);

            timeOfFlight = goalState.timeOfFlight() * TOFAdjustment;
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            Pose2d newLookaheadPose =
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            
            // early exit if converged (change < 1mm)
            if (i > 0 && lookaheadPose.getTranslation().getDistance(newLookaheadPose.getTranslation()) < 0.001) {
                lookaheadPose = newLookaheadPose;
                break;
            }
            lookaheadPose = newLookaheadPose;
        }

        // Get the field angle relative to the target pose
        targetAngle = target.minus(lookaheadPose.getTranslation()).getAngle();

        targetAngle = target.minus(lookaheadPose.getTranslation()).getAngle().minus(new Rotation2d(turret.getPositionRad()));


        // Pitch is in radians
        hoodAngle = goalState.pitch();
        hoodSetpoint = MathUtil.clamp(Units.radiansToDegrees(hoodAngle), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.LOOP_TIME);
        lastHoodAngle = hoodAngle;

        distanceFromTarget = target.getDistance(lookaheadPose.getTranslation());
    }

    public void updateDrivePose(){
        Pose2d currentPose = drive.getPose();

        drivepose = new Pose2d(
            currentPose.getTranslation(), 
            // Uncomment this if robot is backwards
            currentPose.getRotation()//.plus(new Rotation2d(Math.PI))
        );
        ChassisSpeeds robotRelVel = drive.getChassisSpeeds();

        // Add a phase delay extrapolation component for latency delay
        drivepose.exp(       //TODO this is a problem, drivepose.exp returns a Pose2d, should be drivepose = drivepose.exp(...)
            new Twist2d(
                robotRelVel.vxMetersPerSecond * phaseDelay,
                robotRelVel.vyMetersPerSecond * phaseDelay,
                robotRelVel.omegaRadiansPerSecond * phaseDelay)); 
    }

    /**
     * Stops and stows all subsystems involved in the command
     */
    public void stowEverything(){
        hood.setFieldRelativeTarget(Rotation2d.fromDegrees(HoodConstants.MAX_ANGLE), 0.0);
        shooter.setShooter(0.0);
        //spindexer.stopSpindexer();
    }

    // shoot higher
    public void bumpUpHoodOffset() {
        hoodOffset += 1.0; // 1 degree
    }

    // shoot lower
    public void bumpDownHoodOffset() {
        hoodOffset -= 1.0; // 1 degree
    }

    // aim more left
    public void bumpUpTurretOffset() {
        turretOffset += 2.5; // 2.5 degree
    }
    // aim more right
    public void bumpDownTurretOffset() {
        turretOffset -= 2.5; // 2.5 degree
    }

    @Override
    public void execute() {
        updateDrivePose();

        // Phase manager stuff
        phaseManager.update(drivepose, shooter, turret);
        target = phaseManager.getTarget(drivepose);

        updateSetpoints(drivepose);

        if (phaseManager.isIdle()) {
            stowEverything();
        } else {
            drive.setAlignAngle(targetAngle.getRadians());

            boolean shuttling = !target.equals(FieldConstants.getHubTranslation().toTranslation2d()); // if we're aiming at the hub, we're not shuttling

            // shuttling will move the hood so its angles very close to max (less arch)
            if (shuttling) {
                hood.setFieldRelativeTarget(Rotation2d.fromDegrees(ShuttleInterpolation.newHoodMap.get(distanceFromTarget)), hoodVelocity);
            } else {
                hood.setFieldRelativeTarget(Rotation2d.fromDegrees(ShotInterpolation.newHoodMap.get(distanceFromTarget)), hoodVelocity);
            }
            
            double x = drivepose.getX(); // compared as meters
            double y = drivepose.getY();

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

        }


    }

    @Override
    public void end(boolean interrupted) {
      stowEverything();

      drive.setIsAlign(false);

      turret.locked = false;

    }

    @Override
    public void initialize() {
      drive.setIsAlign(true);
      turret.locked = true;
    }

}
