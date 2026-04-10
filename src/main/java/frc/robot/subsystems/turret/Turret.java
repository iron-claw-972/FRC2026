package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Turret extends SubsystemBase implements TurretIO{
	// Super low magnitude filter for the position to make it less jittery
	private final LinearFilter setpointFilter = LinearFilter.singlePoleIIR(0.02, 0.02);

    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  	public boolean locked = false;
	
	private final TalonFX motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.CANIVORE_SUB);

	private TalonFXSimState simState;
	private SingleJointedArmSim turretSim;

	private Rotation2d goalAngle = Rotation2d.kZero;
	private double goalVelocityRadPerSec = 0.0;

	private final MotionMagicVoltage mmVoltageRequest = new MotionMagicVoltage(0);

	public Turret() {
		motor.setNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
		config.Slot0.kP = 12.0; 
		config.Slot0.kS = 0.1; // Static friction compensation
		config.Slot0.kV = 0.12; // Adjusted kV for the gear ratio
		config.Slot0.kD = 0.0; // The "Braking" term to stop overshoot

		var mm = config.MotionMagic;
		mm.MotionMagicCruiseVelocity = Units.radiansToRotations(TurretConstants.MAX_VELOCITY) * TurretConstants.GEAR_RATIO;
		mm.MotionMagicAcceleration = Units.radiansToRotations(TurretConstants.MAX_ACCELERATION) * TurretConstants.GEAR_RATIO; // Lowered for belt safety
		mm.MotionMagicJerk = 0; // Set to > 0 for "S-Curve" smoothing if needed
        motor.getConfigurator().apply(config);

		setCurrentLimits(TurretConstants.NORMAL_CURRENT_LIMIT);

		if (RobotBase.isSimulation()) {
			simState = motor.getSimState();
			turretSim = new SingleJointedArmSim(
					edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
					TurretConstants.GEAR_RATIO,
					0.01,
					0.15,
					Units.degreesToRadians(TurretConstants.MIN_ANGLE),
					Units.degreesToRadians(TurretConstants.MAX_ANGLE),
					false,
					0.0);
		}

		SmartDashboard.putData("Set Locked", new InstantCommand(() -> {locked = !locked;}));
		//motor.setPosition(Units.degreesToRotations(238.86) * TurretConstants.GEAR_RATIO);

		motor.setPosition(0.0);
	}

	/**
	 * Sets the setpoint position and velocity of the turret. Call in command execute.
	 * @param angle
	 * @param velocityRadPerSec
	 */
	public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
		goalAngle = angle;
		goalVelocityRadPerSec = velocityRadPerSec;
	}

	public void resetTurretPosition() {
		inputs.positionDeg = 0.0;
	}

	/**
	 * @return If the turret is at setpoint with tolerance of 10 degrees
	 */
	public boolean atSetpoint() {
		if (locked) return true;
		return Math.abs(goalAngle.getRadians() - getPositionRad()) < Units.degreesToRadians(10.0);
	}

	/**
	 * @return Posiiton of the turret in radians
	 */
	public double getPositionRad() {
		return Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
	}

	/**
	 * @return Posiiton of the turret in degrees
	 */
	public double getPositionDeg() {
		return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
	}

	@Override
	public void periodic() {
		updateInputs();

		// Sets motor control with feedforward
		motor.setControl(mmVoltageRequest
		.withPosition(motorGoalRotations)
		.withFeedForward(robotTurnCompensation)
		.withEnableFOC(true));
	}

	@Override
	public void simulationPeriodic() {
		turretSim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
		turretSim.update(Constants.LOOP_TIME);

		simState.setRawRotorPosition(
				Units.radiansToRotations(turretSim.getAngleRads()) * TurretConstants.GEAR_RATIO);

		simState.setRotorVelocity(
				Units.radiansToRotations(turretSim.getVelocityRadPerSec()) * TurretConstants.GEAR_RATIO);
	}

	@Override
	public void updateInputs() {
		inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
		inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
		inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
		inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
	}

	/**
     * sets supply and stator current limits
     * @param limit the current limit for stator and supply current
     */
    public void setCurrentLimits(double limit) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(limit)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);

		if(limit == TurretConstants.NORMAL_CURRENT_LIMIT){
			limits.SupplyCurrentLowerLimit = TurretConstants.CALIBRATION_CURRENT_LIMIT;
			limits.SupplyCurrentLowerTime = 1.0; // Set to lower limit if over 1 second
		}

        motor.getConfigurator().apply(limits);
    }
}
