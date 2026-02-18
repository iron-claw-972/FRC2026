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
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
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
    private Spindexer spindexer;

    private enum WantedState {
        IDLE,
        SHOOTING,
        PASSING
    }

    private enum CurrentState {
        IDLE,
        STARTING_UP,
        TURNING_AROUND,
        SHOOTING,
        PASSING
    }

    private WantedState wantedState = WantedState.IDLE;
    private CurrentState currentState = CurrentState.IDLE;

    private void updateStates(){

    }

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

    private final double phaseDelay = 0.03; // Extrapolation delay due to latency

    private Translation2d target = FieldConstants.getHubTranslation().toTranslation2d();

    public AutoShootCommand(Turret turret, Drivetrain drivetrain, Hood hood, Shooter shooter, Spindexer spindexer) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
        this.spindexer = spindexer;
        drivepose  = drivetrain.getPose();
        //drivepose  = new Pose2d(drivepose.getTranslation(), drivepose.getRotation().plus(new Rotation2d(Math.PI)));

        goalState = ShooterPhysics.getShotParams(
				FieldConstants.getHubTranslation().minus(new Translation3d(drivepose.getTranslation())),
				8.0); // Random initial goalState to prevent it being null
        
        addRequirements(turret);
    }

    public void updateSetpoints(Pose2d drivepose) {
        Pose2d turretPosition = drivepose.transformBy(new Transform2d((new Translation2d(TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getX(),TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getY())), new Rotation2d()));
        
        // If the robot is moving, adjust the target position based on velocity
        ChassisSpeeds robotRelVel = drivetrain.getChassisSpeeds();
        ChassisSpeeds fieldRelVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelVel, drivetrain.getYaw());

        // Rotational adjustment is not being used, since turret is in center of robot
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

        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;

        /*
         * Loop (20) until lookaheadPose converges BECAUSE -->
         * If you're 8m away (t = 1.0s) and moving at 2m/s towards target, you calculate for 6m (t = 0.8s)
         * At 6m, we run assuming t = 0.8 but then the 6m isn't correct since it was derived using t = 1.0s
         * So we make a bunch of guesses until it converges
         */
        for (int i = 0; i < 20; i++) {
            Translation3d lookahead3d = new Translation3d(lookaheadPose.getX(), lookaheadPose.getY(), TurretConstants.DISTANCE_FROM_ROBOT_CENTER.getZ());
            Translation3d target3d = new Translation3d(target.getX(), target.getY(), FieldConstants.getHubTranslation().getZ()); // Add if statement so that it's only when it's shooting
            goalState = ShooterPhysics.getShotParams(
				target3d.minus(lookahead3d),
				2.0);

            timeOfFlight = goalState.timeOfFlight();
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose =
                new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
        }

        // Get the field angle relative to the target pose
        turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        if (lastTurretAngle == null) {
            lastTurretAngle = turretAngle;
        }

        // Take the filtered average as the turret's velocity when robot is moving translationally
        turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / Constants.LOOP_TIME);
        
        lastTurretAngle = turretAngle;

        Logger.recordOutput("Lookahead Pose", lookaheadPose);

        // Subtract the rotational angle of the robot from the setpoint
        double adjustedTurretSetpoint = MathUtil.angleModulus(turretAngle.getRadians() - drivepose.getRotation().getRadians());

        // Shortest path
        double error = MathUtil.inputModulus(Units.radiansToDegrees(adjustedTurretSetpoint) - Units.radiansToDegrees(turret.getPositionRad()), -180, 180);
        double potentialSetpoint = Units.radiansToDegrees(turret.getPositionRad()) + error;

        // Stay within +/- 200 -- if  shortest path is past 200, we go long way around
        double turretRange = TurretConstants.MAX_ANGLE - TurretConstants.MIN_ANGLE;
        if (potentialSetpoint > turretRange/2) {
            potentialSetpoint -= 360;
        } else if (potentialSetpoint < -turretRange/2) {
            potentialSetpoint += 360;
        }

        turretSetpoint = potentialSetpoint;

        // Pitch is in radians
        hoodAngle = goalState.pitch();
        hoodSetpoint = MathUtil.clamp(Units.radiansToDegrees(hoodAngle), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.LOOP_TIME);
        lastHoodAngle = hoodAngle;

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
        drivepose.exp(
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
        shooter.setShooter(ShotInterpolation.exitVelocityMap.get(goalState.exitVel()));

        SmartDashboard.putNumber("Turret Calculated Setpoint", turretSetpoint);
        SmartDashboard.putNumber("Hood Calculate Setpoint", hoodSetpoint);
        SmartDashboard.putNumber("Shooter Calculate Velocity", goalState.exitVel());
        System.out.println("COMMAND IS WORKINNGGG");

        /** Spindexer Stuff!! */
        if(spindexer != null){
            spindexer.maxSpindexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Set the turret and hood to a safe position when the command ends
        turret.setFieldRelativeTarget(new Rotation2d(0.0), 0.0);
        shooter.setShooter(0.0);
        hood.setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE)), 0.0);
        if(spindexer != null){
            spindexer.stopSpindexer();
        }
    }

}