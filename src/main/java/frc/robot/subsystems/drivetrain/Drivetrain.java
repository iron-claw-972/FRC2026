package frc.robot.subsystems.drivetrain;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.util.EqualsUtil;
import frc.robot.util.SwerveModulePose;
import frc.robot.util.SwerveStuff.SwerveSetpoint;
import frc.robot.util.Vision.Vision;

/**
 * using the Phoenix 6 {@link SwerveDrivetrain}.
 * 
 * taran says hi
 */
public class Drivetrain extends GeneratedDrivetrain {
    public static final Lock odometryLock = new ReentrantLock();

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });

    private final PIDController xController =
            new PIDController(DriveConstants.TRANSLATIONAL_P, 0, DriveConstants.TRANSLATIONAL_D);
    private final PIDController yController =
            new PIDController(DriveConstants.TRANSLATIONAL_P, 0, DriveConstants.TRANSLATIONAL_D);
    private final PIDController rotationController =
            new PIDController(DriveConstants.HEADING_P, 0, DriveConstants.HEADING_D);

    private SwerveModulePose modulePoses;
    private final Field2d field = new Field2d();

    private Supplier<Pose2d> desiredPoseSupplier = () -> null;
    private Double alignAngle;
    private double currentHeading;
    private boolean driveTurning;
    private boolean stateDeadband = true;
    private boolean optimizeStates = true;
    private boolean trenchAssist;
    private boolean trenchAlign;
    private double centerOfMassHeight;
    private double previousAngularVelocity;
    private Vision vision;
    private boolean visionEnabled;
    private boolean slipped;

    public Drivetrain(Vision vision) {
        this();
        this.vision = vision;
        this.visionEnabled = VisionConstants.ENABLED;
    }

    public Drivetrain() {
        super(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        vision = null;
        visionEnabled = false;
        initialize();
    }

    private void initialize() {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(0.25), Units.degreesToRadians(0.25));
        modulePoses = new SwerveModulePose(this, DriveConstants.MODULE_LOCATIONS);

        PathPlannerLogging.setLogActivePathCallback(activePath -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
        PathPlannerLogging.setLogTargetPoseCallback(targetPose -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        if (vision != null && visionEnabled) {
            updateOdometryVision();
        }

        Logger.recordOutput("Odometry/Robot", getPose());
        Logger.recordOutput("Odometry/module poses", modulePoses.getModulePoses());

        double[] offsets = {DriveConstants.STEER_OFFSET_FRONT_LEFT, DriveConstants.STEER_OFFSET_FRONT_RIGHT, DriveConstants.STEER_OFFSET_BACK_LEFT, DriveConstants.STEER_OFFSET_BACK_RIGHT};
        var modulePosesss = modulePoses.getModulePoses();
        for (int i = 3; i >= 0; i--) {
            offsets[i] = offsets[i] + modulePosesss[i].getRotation().getDegrees();
        }
        for (int i = 0; i < 4; i++) {
            Logger.recordOutput("Drivetrain/Module" + i + "/AbsoluteEncoderPositionDegrees",
                Units.rotationsToDegrees(getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble()));
        }
        Logger.recordOutput("Odometry/offset poses", offsets);

        
        modulePoses.update();
        field.setRobotPose(getPose());
    }

    public void close() {
        if (simNotifier != null) {
            simNotifier.close();
        }
        super.close();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = ChassisSpeeds.discretize(xSpeed, ySpeed, rot, Constants.LOOP_TIME);
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        }
        setChassisSpeeds(speeds, isOpenLoop);
    }

    public void driveHeading(double xSpeed, double ySpeed, double heading, boolean fieldRelative) {
        double rot = rotationController.calculate(getYaw().getRadians(), heading);
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());
        }
        setChassisSpeeds(speeds, false);
    }

    public void driveWithPID(double x, double y, double rot) {
        Pose2d pose = getPose();
        drive(
                xController.calculate(pose.getX(), x),
                yController.calculate(pose.getY(), y),
                rotationController.calculate(pose.getRotation().getRadians(), rot),
                true,
                false);
    }

    public void updateOdometryVision() {
        if (vision == null || !visionEnabled) {
            return;
        }
        vision.updateInputs();
        vision.updateOdometry(
                getPose(),
                timestamp -> getPose().getRotation().getRadians(),
                slipped,
                (pose, timestamp, standardDeviations) ->
                        addVisionMeasurement(pose, timestamp, standardDeviations));
        modulePoses.update();
        slipped = modulePoses.slipped();
    }

    public void stop() {
        setControl(new SwerveRequest.SwerveDriveBrake());
        currentSetpoint = new SwerveSetpoint(
                new ChassisSpeeds(), Arrays.stream(getModuleStates()).map(state ->
                        new SwerveModuleState(0, state.angle)).toArray(SwerveModuleState[]::new));
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        ChassisSpeeds discretized = ChassisSpeeds.discretize(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond,
                Constants.LOOP_TIME);
        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(discretized);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED);
        currentSetpoint = new SwerveSetpoint(discretized, states);

        SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(discretized)
                .withDriveRequestType(
                        isOpenLoop
                                ? SwerveModule.DriveRequestType.OpenLoopVoltage
                                : SwerveModule.DriveRequestType.Velocity)
                .withSteerRequestType(SwerveModule.SteerRequestType.Position);
        setControl(request);
    }

    public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        SwerveModuleState[] requested = Arrays.copyOf(states, 4);
        SwerveDriveKinematics.desaturateWheelSpeeds(requested, DriveConstants.MAX_SPEED);
        SwerveModuleState[] applied = new SwerveModuleState[4];
        SwerveDriveState state = getState();

        for (int i = 0; i < 4; i++) {
            SwerveModuleState desired = requested[i];
            if (stateDeadband && Math.abs(desired.speedMetersPerSecond) <= DriveConstants.MAX_SPEED * 0.01) {
                desired = new SwerveModuleState(0, state.ModuleStates[i].angle);
            } else if (optimizeStates) {
                desired.optimize(state.ModuleStates[i].angle);
            }
            applied[i] = desired;
            getModule(i).apply(new SwerveModule.ModuleRequest()
                    .withState(desired)
                    .withDriveRequest(isOpenLoop
                            ? SwerveModule.DriveRequestType.OpenLoopVoltage
                            : SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequest(SwerveModule.SteerRequestType.Position));
        }
        currentSetpoint = new SwerveSetpoint(DriveConstants.KINEMATICS.toChassisSpeeds(applied), applied);
    }

    public void setDriveVoltages(Voltage voltage) {
        double volts = voltage.baseUnitMagnitude();
        for (int i = 0; i < 4; i++) {
            getModule(i).getDriveMotor().setControl(new VoltageOut(volts));
        }
    }

    public void setAngleMotors(Rotation2d[] angles) {
        for (int i = 0; i < 4; i++) {
            getModule(i).apply(new SwerveModule.ModuleRequest()
                    .withState(new SwerveModuleState(0, angles[i]))
                    .withDriveRequest(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequest(SwerveModule.SteerRequestType.Position));
        }
    }

    public double getAngularRate(int id) {
        return getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
    }

    public SwerveModulePosition[] getModulePositions() {
        return getState().ModulePositions;
    }

    public void setStateDeadband(boolean enabled) {
        stateDeadband = enabled;
    }

    public void setOptimized(boolean optimized) {
        optimizeStates = optimized;
    }

    public void setVisionEnabled(boolean enabled) {
        visionEnabled = enabled && vision != null && VisionConstants.ENABLED;
    }

    private boolean isAlign = false;
    public void setIsAlign(boolean isAlign) {
        this.isAlign = isAlign;
    }

    public boolean getIsAlign() {
        return isAlign;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    public SwerveModuleState[] getModuleStates() {
        return getState().ModuleStates;
    }

    public SwerveSetpoint getCurrSetpoint() {
        return currentSetpoint;
    }

    public Rotation2d getYaw() {
        return getPose().getRotation();
    }

    public void setYaw(Rotation2d rotation) {
        resetOdometry(new Pose2d(getPose().getTranslation(), rotation));
    }

    public void resetOdometry(Pose2d pose) {
        currentHeading = pose.getRotation().getRadians();
        resetPose(pose);
        modulePoses.reset();
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return getState().Pose;
    }

    public void setPose(Translation2d pose) {
        resetTranslation(pose);
    }

    public void setAlignAngle(Double newAngle) {
        alignAngle = newAngle;
    }

    public boolean atAlignAngle() {
        if (alignAngle == null) {
            return false;
        }
        double error = Math.abs(Math.IEEEremainder(alignAngle - getYaw().getRadians(), 2 * Math.PI));
        return error < DriveConstants.HEADING_TOLERANCE;
    }

    public double getAlignAngle() {
        return alignAngle == null ? 0 : alignAngle;
    }

    public void onlyUseTags(int[] ids) {
        if (vision != null) {
            vision.onlyUse(ids);
        }
    }

    public boolean canSeeTag() {
        return vision != null && vision.canSeeTag();
    }

    public Pose2d getPoseAt(double timestamp) {
        return getPose();
    }

    public double headingControl(double rot, double xSpeed, double ySpeed) {
        if ((!EqualsUtil.epsilonEquals(getAngularRate(0), 0, 0.0004)
                && EqualsUtil.epsilonEquals(Math.hypot(xSpeed, ySpeed), 0, 0.1))
                || !EqualsUtil.epsilonEquals(rot, 0, 0.0004)) {
            driveTurning = true;
            currentHeading = getYaw().getRadians();
        } else {
            driveTurning = false;
        }
        if (!driveTurning) {
            rotationController.setSetpoint(currentHeading);
            double output = rotationController.calculate(getYaw().getRadians());
            rot = Math.abs(output) > Math.abs(rot) ? output : rot;
        }
        return rot;
    }

    public PIDController getXController() {
        return xController;
    }

    public PIDController getYController() {
        return yController;
    }

    public PIDController getRotationController() {
        return rotationController;
    }

    public void setDesiredPose(Supplier<Pose2d> supplier) {
        desiredPoseSupplier = supplier;
    }

    public void setDesiredPose(Pose2d pose) {
        setDesiredPose(() -> pose);
    }

    public Pose2d getDesiredPose() {
        return desiredPoseSupplier.get();
    }

    public boolean atSetpoint() {
        Pose2d desired = getDesiredPose();
        return desired != null && getPose().getTranslation().getDistance(desired.getTranslation()) < 0.025;
    }

    public SwerveModulePose getSwerveModulePose() {
        return modulePoses;
    }

    public double getAcceleration() {
        double accelX = getPigeon2().getAccelerationX().getValueAsDouble();
        double accelY = getPigeon2().getAccelerationY().getValueAsDouble();
        double angularVelocity = getAngularRate(3);
        double angularAccel = (angularVelocity - previousAngularVelocity) / Constants.LOOP_TIME;
        previousAngularVelocity = angularVelocity;

        double pigeonOffsetX = 0.082677;
        double pigeonOffsetY = 0.030603444;
        double totalX = accelX + angularVelocity * angularVelocity * pigeonOffsetX + angularAccel * pigeonOffsetY;
        double totalY = accelY + angularVelocity * angularVelocity * pigeonOffsetY - angularAccel * pigeonOffsetX;
        return Math.hypot(totalX, totalY);
    }

    @AutoLogOutput(key = "Drivetrain/AccelerationFaults")
    public boolean accelerationOverMax() {
        return getAcceleration() > DriveConstants.MAX_LINEAR_ACCEL;
    }

    public void setCenterOfMass(double height) {
        centerOfMassHeight = height;
    }

    public void alignWheels() {
        setControl(new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.kZero));
    }

    public boolean getTrenchAssist() {
        return trenchAssist;
    }

    public boolean getTrenchAlign() {
        return trenchAlign;
    }

    public void setTrenchAssist(boolean target) {
        trenchAssist = target;
    }

    public void setTrenchAlign(boolean target) {
        trenchAlign = target;
    }

    public void applyNewModuleCurrents(
            double steerCurrentStator,
            double steerCurrentSupply,
            double driveCurrentStator,
            double driveCurrentSupply) {
        CurrentLimitsConfigs steer = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(edu.wpi.first.units.Units.Amps.of(steerCurrentStator))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(edu.wpi.first.units.Units.Amps.of(steerCurrentSupply))
                .withSupplyCurrentLimitEnable(true);
        CurrentLimitsConfigs drive = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(edu.wpi.first.units.Units.Amps.of(driveCurrentStator))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(edu.wpi.first.units.Units.Amps.of(driveCurrentSupply))
                .withSupplyCurrentLimitEnable(true);
        for (int i = 0; i < 4; i++) {
            getModule(i).getSteerMotor().getConfigurator().apply(steer);
            getModule(i).getDriveMotor().getConfigurator().apply(drive);
        }
    }

    public double getSubsystemStatorCurrent() {
        double total = 0;
        for (int i = 0; i < 4; i++) {
            total += getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble();
            total += getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble();
        }
        return total;
    }

    public double getSubsystemSupplyCurrent() {
        double total = 0;
        for (int i = 0; i < 4; i++) {
            total += getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble();
            total += getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble();
        }
        return total;
    }
}
