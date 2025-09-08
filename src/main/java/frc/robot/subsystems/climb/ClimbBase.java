package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.util.ClimbArmSim;

abstract class ClimbBase extends SubsystemBase {

	private static final double STARTING_POSITION = 0;
	private static final double EXTEND_POSITION = 2;
	private static final double CLIMB_POSITION = -0.83;

	private static final double VERSA_PLANETARY_GEAR_RATIO = 1;
	private static final double CLIMB_GEAR_RATIO = 60./1.;
	private static final double TOTAL_GEAR_RATIO = VERSA_PLANETARY_GEAR_RATIO * CLIMB_GEAR_RATIO;

	private final PIDController pid = new PIDController(2.5, 0, 0);
	private TalonFX motor = new TalonFX(IdConstants.CLIMB_MOTOR, Constants.CANIVORE_CAN);

	private final DCMotor climbGearBox = DCMotor.getKrakenX60(1);
	private final TalonFXSimState encoderSim;
	private final Mechanism2d simulationMechanism = new Mechanism2d(3, 3);
	private final MechanismRoot2d mechanismRoot = simulationMechanism.getRoot("Climb", 1.5, 1.5);
	private final MechanismLigament2d simLigament = mechanismRoot
			.append(new MechanismLigament2d("angle", 1, STARTING_POSITION, 4, new Color8Bit(Color.kAntiqueWhite)));

	private ClimbArmSim climbSim;
	private double power;
	private boolean resetting = false;
}
