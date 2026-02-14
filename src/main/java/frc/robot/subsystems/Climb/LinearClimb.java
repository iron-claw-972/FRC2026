package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.Climb.ClimbConstants;
import frc.robot.constants.swerve.DriveConstants;

public class LinearClimb extends SubsystemBase{
    private TalonFX motor;

    private double rotationalSetpoint = 0;

    private static PIDController pid = new PIDController(0.1, 0, 0);

    private double kP = 1.0;
    private double kI = 0.0;
    private double kD = 0.0;  

    private double SpringResistance = -0.1;
    private double RobotResistance = (DriveConstants.ROBOT_MASS * Constants.GRAVITY_ACCELERATION * ClimbConstants.RADIUS)/ClimbConstants.CLIMB_GEAR_RATIO;

    private final DCMotor motorConstant = DCMotor.getKrakenX44(1);
    
    private double gearRatio = ClimbConstants.CLIMB_GEAR_RATIO;

    private MotionMagicVoltage voltageRequest = new MotionMagicVoltage(Units.degreesToRotations(0) * gearRatio); // gear ratio


    // ElevatorFeedforward feedforward = new ElevatorFeedforward(0, (((DriveConstants.ROBOT_MASS * Constants.GRAVITY_ACCELERATION * ClimbConstants.RADIUS) / gearRatio) / motorConstant.KtNMPerAmp) * motorConstant.rOhms, 0, 0);
    // ElevatorFeedforward feedforward = new ElevatorFeedforward(0, SpringResistance, 0, 0);

    public LinearClimb() {
        motor = new TalonFX(IdConstants.CLIMB_MOTOR_ID);//, Constants.RIO_CAN);
        pid.setTolerance(0.2);

        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits = new CurrentLimitsConfigs();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 5.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 5.0;

        // config.Slot0.kS = 0.0; // Static friction compensation (should be >0 if friction exists)
        // config.Slot0.kG = 0.0; // Gravity compensation
        // config.Slot0.kV = 1; // Velocity gain: 1 rps -> 0.12V
        // config.Slot0.kA = 0.3; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        
        // config.Slot0.kP = kP; // If position error is 1 rotation, apply 10V
        // config.Slot0.kI = kI; // Integral term (usually left at 0 for MotionMagic)
        // config.Slot0.kD = kD; // Derivative term (used to dampen oscillations)
        
        // MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(ClimbConstants.MAX_VELOCITY) * gearRatio; // max velocity * gear ratio
        // motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(ClimbConstants.MAX_ACCELERATION) * gearRatio; // max Acceleration * gear ratio
        // SmartDashboard.putNumber("velocity", Units.radiansToRotations(ClimbConstants.MAX_VELOCITY) * gearRatio);
        // SmartDashboard.putNumber("acceleration", Units.radiansToRotations(ClimbConstants.MAX_ACCELERATION) * gearRatio);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
        SmartDashboard.putData("Go Up", new InstantCommand(() -> goUp()));
        SmartDashboard.putData("Go Down", new InstantCommand(() -> goDown()));
        SmartDashboard.putData("Climb", new InstantCommand(() -> climb()));
        SmartDashboard.putNumber("Position", getPosition());

        // motor.setPosition((ClimbConstants.MAX_HEIGHT/(2 * Math.PI * ClimbConstants.RADIUS)) * gearRatio);
        motor.setPosition(0);
    }

    /**
    * set the setpoint for the pid
    * @param setpoint in rotations
    */
    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
        // setpoint = MathUtil.clamp(setpoint, ClimbConstants.MIN_HEIGHT, ClimbConstants.MAX_HEIGHT);

        // // rotationalSetpoint = (setpoint / (2 * Math.PI * ClimbConstants.RADIUS)) * gearRatio;
        // rotationalSetpoint = -45/2.25;
        // System.out.println(rotationalSetpoint);

        // // motor.setControl(voltageRequest.withPosition(rotationalSetpoint).withFeedForward(feedforward.calculate(0)));
        // motor.setControl(voltageRequest.withPosition(rotationalSetpoint));
    }

    public boolean atSetPoint(){
        return pid.atSetpoint();
        // return Math.abs(motor.getPosition().getValueAsDouble() - rotationalSetpoint) < 3.0;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public void goUp() {
        // feedforward.setKg(SpringResistance);
        setSetpoint(1);
    }

    public void goDown() {
        // feedforward.setKg(RobotResistance  - SpringResistance);
        setSetpoint(ClimbConstants.MIN_HEIGHT);
    }

    public void climb() {
        setSetpoint(ClimbConstants.CLIMB_HEIGHT);
    }

    public void periodic() {
        double power = pid.calculate(motor.getPosition().getValueAsDouble());
        power = MathUtil.clamp(power, -0.2, 0.2);
        motor.set(power);
        SmartDashboard.putNumber("Position", getPosition());
    }
}