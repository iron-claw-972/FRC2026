package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Turret extends SubsystemBase {

    /* ---------------- Constants ---------------- */

    private static final double MIN_ANGLE_RAD =
        Units.degreesToRadians(TurretConstants.MIN_ANGLE);
    private static final double MAX_ANGLE_RAD =
        Units.degreesToRadians(TurretConstants.MAX_ANGLE);

    private static final double MAX_VEL_RAD_PER_SEC = 16.0;
    private static final double MAX_ACCEL_RAD_PER_SEC2 = 1e7;

    private static final double VERSA_RATIO = 5.0;
    private static final double TURRET_RATIO = 140.0 / 10.0;
    private static final double GEAR_RATIO = VERSA_RATIO * TURRET_RATIO;

    /* ---------------- Hardware ---------------- */

    private final TalonFX motor =
        new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.RIO_CAN);

    private TalonFXSimState simState;
    private SingleJointedArmSim turretSim;

    private final PositionVoltage positionRequest = new PositionVoltage(0.0);

    /* ---------------- Control ---------------- */

    private final TrapezoidProfile profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                MAX_VEL_RAD_PER_SEC,
                MAX_ACCEL_RAD_PER_SEC2));

    private State setpoint = new State();

    private Rotation2d goalAngle = Rotation2d.kZero;
    private double goalVelocityRadPerSec = 0.0;
    private double lastGoalRad = 0.0;

    /* ---------------- Gains (ALPHABOT, MA-converted) ---------------- */

    // 3500 / rad  → × 2π
    private static final double kP = 22_000.0;

    // 150 / (rad/s) → × 2π
    private static final double kD = 943.0;

    /* ---------------- Visualization ---------------- */

    private final Mechanism2d mech = new Mechanism2d(100, 100);
    private final MechanismRoot2d root = mech.getRoot("turret", 50, 50);
    private final MechanismLigament2d ligament =
        root.append(new MechanismLigament2d("barrel", 30, 0));

    /* ---------------- Constructor ---------------- */

    public Turret() {
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setPosition(0.0);

        motor.getConfigurator().apply(
            new Slot0Configs()
                .withKP(kP)
                .withKD(kD));

        motor.getConfigurator().apply(
            new com.ctre.phoenix6.configs.TalonFXConfiguration() {{
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            }});

        if (RobotBase.isSimulation()) {
        simState = motor.getSimState();
        turretSim =
            new SingleJointedArmSim(
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
                GEAR_RATIO,
                0.01,
                0.15,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD,
                false,
                0.0);
        }

        SmartDashboard.putData("Turret Mech", mech);
    }

    /* ---------------- Public API ---------------- */

    public void setFieldRelativeTarget(Rotation2d angle, double velocityRadPerSec) {
        goalAngle = angle;
        goalVelocityRadPerSec = velocityRadPerSec;
    }

    public boolean atGoal() {
        return Math.abs(setpoint.position - lastGoalRad) < Units.degreesToRadians(1.5);
    }

    public double getPositionRad() {
        return Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) / GEAR_RATIO;
    }

    /* ---------------- Periodic ---------------- */

    @Override
    public void periodic() {

        double robotRelativeGoal = goalAngle.getRadians();

        // --- MA-style continuous wrap selection ---
        double best = lastGoalRad;
        boolean found = false;

        for (int i = -2; i <= 2; i++) {
        double candidate = robotRelativeGoal + 2.0 * Math.PI * i;
        if (candidate < MIN_ANGLE_RAD || candidate > MAX_ANGLE_RAD) continue;

        if (!found || Math.abs(candidate - lastGoalRad) < Math.abs(best - lastGoalRad)) {
            best = candidate;
            found = true;
        }
        }

        lastGoalRad = best;

        // --- Profile in MECHANISM SPACE ---
        State goalState =
            new State(
                MathUtil.clamp(best, MIN_ANGLE_RAD, MAX_ANGLE_RAD),
                goalVelocityRadPerSec);

        setpoint =
            profile.calculate(
                Constants.LOOP_TIME,
                setpoint,
                goalState);

        // --- Convert to MOTOR SPACE ---
        double motorPosRot =
            Units.radiansToRotations(setpoint.position) * GEAR_RATIO;

        double motorVelRotPerSec =
            Units.radiansToRotations(setpoint.velocity) * GEAR_RATIO;

        // --- Position + velocity feedforward (MA-style) ---
        motor.setControl(
            positionRequest
                .withPosition(motorPosRot)
                .withVelocity(motorVelRotPerSec));

        // --- Visualization ---
        ligament.setAngle(Units.radiansToDegrees(getPositionRad()));

        SmartDashboard.putNumber("Turret/GoalDeg",
            Units.radiansToDegrees(best));
        SmartDashboard.putNumber("Turret/SetpointDeg",
            Units.radiansToDegrees(setpoint.position));
    }

    /* ---------------- Simulation ---------------- */

    @Override
    public void simulationPeriodic() {
        turretSim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
        turretSim.update(Constants.LOOP_TIME);

        simState.setRawRotorPosition(
            Units.radiansToRotations(turretSim.getAngleRads()) * GEAR_RATIO);

        simState.setRotorVelocity(
            Units.radiansToRotations(turretSim.getVelocityRadPerSec()) * GEAR_RATIO);
    }
}
