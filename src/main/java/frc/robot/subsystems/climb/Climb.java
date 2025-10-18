package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.util.ClimbArmSim;

public class Climb extends SubsystemBase {

	private static final double START_DEG = 0;
	private static final double EXT_DEG = 90.0;
	private static final double CLIMB_DEG = -37.35;

	// Motors
	private final PIDController pid = new PIDController(2.5, 0.0, 0.0);

	private TalonFX motor = new TalonFX(IdConstants.CLIMB_MOTOR, Constants.CANIVORE_CAN);
	private final DCMotor climbGearBox = DCMotor.getKrakenX60(1);
	private TalonFXSimState encoderSim;

	// Mechanism2d display
	private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
	private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Climb", 1.5, 1.5);
	private final MechanismLigament2d simLigament = mechanismRoot.append(
			new MechanismLigament2d("angle", 1, START_DEG, 4, new Color8Bit(Color.kAntiqueWhite)));

	// Gear ratios
	private final double versaPlanetaryGearRatio = 1.0;
	private final double climbGearRatio = 60.0 / 1.0;
	private final double totalGearRatio = versaPlanetaryGearRatio * climbGearRatio;

	private ClimbArmSim climbSim;
	private double power;
	private boolean resetting = false;

	// Object for data logging
	private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

	// Constructor to initialize subsystem
	public Climb() {
        if (RobotBase.isSimulation()) {
            encoderSim = motor.getSimState();
            encoderSim.setRawRotorPosition(Units.degreesToRotations(START_DEG)*totalGearRatio);

            climbSim = new ClimbArmSim(
                climbGearBox, 
                totalGearRatio, 
                0.1, 
                0.127, 
                0, //min angle 
                Units.degreesToRadians(90), //max angle
                true, 
                Units.degreesToRadians(START_DEG),
                60
                );

            climbSim.setIsClimbing(true);
            SmartDashboard.putData("Climb Display", simulationMechanism);
        }

        pid.setIZone(1);

        pid.setSetpoint(Units.degreesToRadians(START_DEG));

        motor.setPosition(Units.degreesToRotations(START_DEG)*totalGearRatio);
        motor.setNeutralMode(NeutralModeValue.Brake);
        //SmartDashboard.putData("Climb PID", pid);
    }

	// Runs repeatedly every 20ms
	@Override
	public void periodic() {

		inputs.measuredPositionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble() / totalGearRatio);
		inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
		Logger.processInputs("Climb", inputs);

		double motorPosition = motor.getPosition().getValueAsDouble();
		double currentPosition = Units.rotationsToRadians(motorPosition / totalGearRatio);
		power = pid.calculate(currentPosition);

		if (resetting) {
			power = -0.1;
		}

		// Logs current and expected position of motor for debugging
		Logger.recordOutput("Climb/Motor Power", power);
		Logger.recordOutput("Climb/positionDeg", getAngle());
		Logger.recordOutput("Climb/setpointDeg", Units.radiansToDegrees(pid.getSetpoint()));

		// Restricts motor power (from -1 to 1)
		motor.set(MathUtil.clamp(power, -1, 1));
	}

	// Runs repeatedly in simulation every 20ms
	@Override
    public void simulationPeriodic() {
        climbSim.setInput(power * Constants.ROBOT_VOLTAGE);
        climbSim.update(Constants.LOOP_TIME);

        double climbRotations = Units.radiansToRotations(climbSim.getAngleRads());
        encoderSim.setRawRotorPosition(climbRotations * totalGearRatio);

        simLigament.setAngle(Units.radiansToDegrees(getAngle()));
    }

	/**
	 * Sets the motor to an angle from 0-90 degrees
	 * 
	 * @param angle in degrees
	 */
	public void setAngle(double angle) {
		pid.reset();
		pid.setSetpoint(Units.degreesToRadians(angle));
	}

}

	/**
	 * Gets the current position of the motor in degrees
	 * 
	 * @return The angle in degrees
	 */
	public double getAngle() {
		return inputs.measuredPositionDeg;
	}

	/**
	 * Turns the motor to 90 degrees (extended position)
	 */
	public void extend() {
		setAngle(EXT_DEG);
	}

	/**
	 * Turns the motor to -37.35 degrees (climb position)
	 */
	public void climb() {
		setAngle(CLIMB_DEG);
	}

	/**
	 * Turns the motor to 0 degrees (stow position)
	 */
	public void stow() {
		setAngle(START_DEG);
	}

	public void reset(boolean resetting) {
		this.resetting = resetting;
		if (!resetting) {
			motor.setPosition(Units.degreesToRotations(CLIMB_DEG) * totalGearRatio);

			pid.setSetpoint(Units.degreesToRadians(START_DEG));
			pid.reset();
		}
	}

	public double getCurrent() {
		return motor.getStatorCurrent().getValueAsDouble();
	}

	// Might not work
	public Mechanism2d getMech2d() {
		return simulationMechanism;
	}

	/**
	 * Gets the estimated angle of the climb.
	 * This is slightly inaccurate since it assumes the climb rotates exactly 90
	 * degrees and the motor position is proportional to the climb position
	 * Used only for 3D display, not exact for control.
	 */
	public double getEstimatedClimbAngle() {
		return (getAngle() / (EXT_DEG - CLIMB_DEG) * Math.PI / 2);
	}
}