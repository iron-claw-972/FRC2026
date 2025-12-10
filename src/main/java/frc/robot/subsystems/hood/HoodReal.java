package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IdConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

// TODO:
// check if NaN is because of drivetrain or velocity being too low
// get sim to work of course
// fix config for motion magic
// tune for gravity and stuff
// implement live odometry distance reading instead of setting manually

public class HoodReal extends HoodBase implements HoodIO {
    final private TalonFX motor;
    private double position;
    private double velocity;
    double power;
    
    private PIDController pid = new PIDController(0.2, 0.0, 0.05);

    // Hood gear ratio from ShooterConstants

    private SingleJointedArmSim hoodSim;
    private static final DCMotor hoodMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(Units.degreesToRotations(HoodConstants.START_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);
    private double setpoint = HoodConstants.START_ANGLE;
    public double distance = HoodConstants.START_DISTANCE;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("hoodMotor", 25, 0));

    // for calculating angle

    private final HoodInputsIOAutoLogged inputs = new HoodInputsIOAutoLogged();

    public HoodReal() {
        // allocate the motor
        motor = new TalonFX(IdConstants.HOOD_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
        
        updateInputs();

        pid.setTolerance(Units.degreesToRadians(3));

        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();

            hoodSim = new SingleJointedArmSim(
                hoodMotorSim,
                HoodConstants.HOOD_GEAR_RATIO,
                0.01 * 0.01 * 5,
                0.10,
                Units.degreesToRadians(-360),
                Units.degreesToRadians(360),
                false,
                HoodConstants.START_ANGLE
            );
        }

        // motor position at power up
        motor.setPosition(Units.degreesToRotations(HoodConstants.START_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);
        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();

        limitConfig.StatorCurrentLimit = 2; // 120
        limitConfig.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(limitConfig);

        config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        config.Slot0.kG = 0.25; // Gravity compensation
        config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        
        // CHATGPT SAYS THIS IS WRONG
        config.Slot0.kP = Units.radiansToRotations(1 * 12); // If position error is 2.5 rotations, apply 12V (0.5 * 2.5 * 12V)
        
        config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        // Recommened from chatGPT:
        // config.Slot0.kP = 8.0;
        // config.Slot0.kI = 0.0;
        // config.Slot0.kD = 0.2;
        // config.Slot0.kS = 0.1;
        // config.Slot0.kG = 0.25;   // flip sign if hood drops
        // config.Slot0.kV = 0.12;


        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(HoodConstants.MAX_VELOCITY) * HoodConstants.HOOD_GEAR_RATIO;
        motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(HoodConstants.MAX_ACCELERATION) * HoodConstants.HOOD_GEAR_RATIO;
        //TODO: find which direction is positive
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        motor.getConfigurator().apply(config);

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MAX_ANGLE) * HoodConstants.HOOD_GEAR_RATIO;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MIN_ANGLE) * HoodConstants.HOOD_GEAR_RATIO;

        SmartDashboard.putData("hood", mechanism2d);
        SmartDashboard.putData("PID", pid);
        
        // SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setSetpoint(90)));
        // SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setSetpoint(180)));
        // SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        // SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setSetpoint(270)));

        SmartDashboard.putData("Move hood for a distance of 3 meters", new InstantCommand(() -> setToCalculatedAngle(HoodConstants.INITIAL_VELOCTIY, HoodConstants.TARGET_HEIGHT, 3)));
        SmartDashboard.putData("Move hood for a distance of 4 meters", new InstantCommand(() -> setToCalculatedAngle(HoodConstants.INITIAL_VELOCTIY, HoodConstants.TARGET_HEIGHT, 4)));
        SmartDashboard.putData("Move hood for a distance of 5 meters", new InstantCommand(() -> setToCalculatedAngle(HoodConstants.INITIAL_VELOCTIY, HoodConstants.TARGET_HEIGHT, 5)));
        SmartDashboard.putData("Move hood for a distance of 6 meters", new InstantCommand(() -> setToCalculatedAngle(HoodConstants.INITIAL_VELOCTIY, HoodConstants.TARGET_HEIGHT, 6)));
   
        SmartDashboard.putData("Recalibrate Hood", new InstantCommand(() -> resetDueToSlippingError()));

        SmartDashboard.putData("Move to max angle", new InstantCommand(() -> setSetpoint(HoodConstants.MAX_ANGLE)));
        SmartDashboard.putData("Move to min angle", new InstantCommand(() -> setSetpoint(HoodConstants.MIN_ANGLE)));

        SmartDashboard.putNumber("Hood Position", getPosition());
        SmartDashboard.putNumber("Hood Setpoint", getSetpoint());
    }

    public void setSetpoint(double setpoint) {
        double error = MathUtil.inputModulus(setpoint - getPosition(), -180.0, 180.0);
        double shortestDeg = getPosition() + error;
        this.setpoint = shortestDeg;
        
        // command the controller with the *wrapped* target
        double motorTargetRotations = Units.degreesToRotations(shortestDeg) * HoodConstants.HOOD_GEAR_RATIO;
        motor.setControl(voltageRequest.withPosition(motorTargetRotations));
        
    }
    

    public double getPosition() {
        return position;
    }
    
    public double getSetpoint() {
        return setpoint;
    }

    // assuming we are aligned of course, but I need to switch if we don't have a turret
    // technically, we should have the drivetrain now the distance at all times, and we can grab that instead
    public double calculateDistanceToTarget(Pose2d robotPose) {
        double dx = HoodConstants.TARGET_POSITION.getX() - robotPose.getX();
        double dy = HoodConstants.TARGET_POSITION.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    public void aimToTarget(Pose2d robotPose) {
        setToCalculatedAngle(HoodConstants.INITIAL_VELOCTIY, HoodConstants.TARGET_HEIGHT, calculateDistanceToTarget(robotPose));
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getVelocity() {
        return velocity/HoodConstants.HOOD_GEAR_RATIO;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - setpoint) < 1.0;
    }

    @Override
    public void periodic() {
        updateInputs();
        //try find a way to do with motion magic
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / HoodConstants.HOOD_GEAR_RATIO;
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
        
        //power = pid.calculate(Units.degreesToRadians(getPosition()));
        //motor.set(power);
        System.out.println(position + " degrees");
        ligament2d.setAngle(position);
        
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    boolean simInitialized = false;
    @Override
    public void simulationPeriodic() {
        //double voltsMotor = power * 12;
        double voltsMotor = motor.getMotorVoltage().getValueAsDouble();
        hoodSim.setInputVoltage(voltsMotor);

        hoodSim.update(Constants.LOOP_TIME);

        double simAngle = hoodSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * HoodConstants.HOOD_GEAR_RATIO;

        encoderSim.setRawRotorPosition(motorRotations); // MUST set position
        encoderSim.setRotorVelocity(hoodSim.getVelocityRadPerSec() * Units.radiansToRotations(1) * HoodConstants.HOOD_GEAR_RATIO);
    }
    
    @Override
    public double calculateAngle(double v0, double U, double R) {
        double g = Constants.GRAVITY_ACCELERATION;
        //TODO: change this
        double shooterHeight = 0.3;
        double h = U - shooterHeight;
    
        double inside = v0*v0*v0*v0 - g * (g * R * R + 2 * h * v0 * v0);
        if (inside < 0) return Double.NaN;
        double sqrtTerm = Math.sqrt(inside);
    
        // match Desmos: v0² + sqrt(...)
        double numerator = (v0 * v0) - sqrtTerm;
    
        double denominator = g * R;
    
        double z = Math.atan(numerator / denominator);
    
        System.out.println("CalculatedAngle = " + z);
        return z;
    }

    

    @Override
    public void setToCalculatedAngle(double initialVelocity, double goalHeight, double goalDistance) {
        double angleRad = calculateAngle(initialVelocity, goalHeight, goalDistance);
        double angleDeg = Units.radiansToDegrees(angleRad);

        System.out.println("AIM angle = " + angleDeg + "degrees");

        // in case we can't reach the target with our velocity:
        if (!Double.isNaN(angleDeg) && !Double.isInfinite(angleDeg)) {
            setSetpoint(MathUtil.clamp(angleDeg, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE));
        } else {
            System.out.println("Angle is not able to reach the target with given velocity.");
        }
    }
    // Intended to be used for the slipping of the bands that are on the gears
    public void resetDueToSlippingError() {
        while (motor.getSupplyCurrent().getValueAsDouble() < HoodConstants.CURRENT_SPIKE_THRESHHOLD) {
            motor.setVoltage(4);;
        }
        position = HoodConstants.START_ANGLE;
    }

    public void updateInputs(){
        inputs.measuredAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / HoodConstants.HOOD_GEAR_RATIO;
    }
}


