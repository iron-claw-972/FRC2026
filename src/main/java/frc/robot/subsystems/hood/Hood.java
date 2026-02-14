package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Hood extends SubsystemBase implements HoodIO{
    private TalonFX motor = new TalonFX(IdConstants.HOOD_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private double MIN_ANGLE_RAD = Units.degreesToRadians(HoodConstants.MIN_ANGLE);
	private double MAX_ANGLE_RAD = Units.degreesToRadians(HoodConstants.MAX_ANGLE);

	private double MAX_VEL_RAD_PER_SEC = 25;
	private double MAX_ACCEL_RAD_PER_SEC2 = 160.0;

    private double GEAR_RATIO = HoodConstants.HOOD_GEAR_RATIO;

    private static final PIDController positionPID = new PIDController(6, 0, 0.2);

    private final TrapezoidProfile profile = new TrapezoidProfile(
		new TrapezoidProfile.Constraints(
				MAX_VEL_RAD_PER_SEC,
				MAX_ACCEL_RAD_PER_SEC2));
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.1, 1. / DCMotor.getKrakenX60(1).KvRadPerSecPerVolt, 0);

	private State setpoint = new State();

	private Rotation2d goalAngle = new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE));
	private double goalVelocityRadPerSec = 0.0;

	private static double kP = 2.0;
	private static double kD = 0.2;

    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(){
		motor.setPosition(Units.degreesToRotations(HoodConstants.MAX_ANGLE) * GEAR_RATIO);
        motor.setNeutralMode(NeutralModeValue.Coast);

        motor.getConfigurator().apply(
				new Slot0Configs()
						.withKP(kP)
						.withKD(kD));

		TalonFXConfiguration config = new TalonFXConfiguration();

        motor.getConfigurator().apply(
				new com.ctre.phoenix6.configs.TalonFXConfiguration() {
					{
						MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
					}
				});

        setpoint = new State(getPositionRad() / GEAR_RATIO, 0.0);

		SmartDashboard.putData("max", new InstantCommand(() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MAX_ANGLE)), 0)));
		SmartDashboard.putData("medium", new InstantCommand(() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians((HoodConstants.MAX_ANGLE + HoodConstants.MIN_ANGLE) / 2)), 0)));
		SmartDashboard.putData("min", new InstantCommand(() -> setFieldRelativeTarget(new Rotation2d(Units.degreesToRadians(HoodConstants.MIN_ANGLE)), 0)));
    }

    public double getPositionRad(){
        return Units.rotationsToRadians(motor.getPosition().getValueAsDouble());
    }

    public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
		goalAngle = angle;
		goalVelocityRadPerSec = velocityRadPerSec;
	}

    @Override
    public void periodic() {
		updateInputs();

        State goalState = new State(
				MathUtil.clamp(goalAngle.getRadians(), MIN_ANGLE_RAD, MAX_ANGLE_RAD),
				goalVelocityRadPerSec);

		setpoint = profile.calculate(
				Constants.LOOP_TIME,
				setpoint,
				goalState);

		double motorVelRotPerSec = Units.radiansToRotations(setpoint.velocity) * GEAR_RATIO;

		double targetVelocity;

        double motorSetpointPosition = (setpoint.position) * GEAR_RATIO;

		targetVelocity = positionPID.calculate(
				motor.getPosition().getValue().in(edu.wpi.first.units.Units.Radians),
				motorSetpointPosition);
			
		targetVelocity += Units.rotationsToRadians(motorVelRotPerSec);

		double voltage = feedForward.calculate(targetVelocity);

		motor.setVoltage(voltage);

        Logger.recordOutput("Hood/Voltage", motor.getMotorVoltage().getValue());
		Logger.recordOutput("Hood/velocitySetpoint", targetVelocity / GEAR_RATIO);
		Logger.recordOutput("Hood/SetpointDeg", Units.radiansToDegrees(setpoint.position) / GEAR_RATIO);

		SmartDashboard.putData("Hood PID", positionPID);

		SmartDashboard.putNumber("Turret Position Deg", Units.radiansToDegrees(getPositionRad()) / GEAR_RATIO);
		Logger.processInputs("Hood", inputs);
	}

    @Override
	public void updateInputs() {
		inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / GEAR_RATIO;
		inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) / GEAR_RATIO;
		inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
	}
}
