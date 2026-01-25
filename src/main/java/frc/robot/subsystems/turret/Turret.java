package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

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

public class Turret extends SubsystemBase implements TurretIO{
    /* ---------------- Constants ---------------- */

    private static final double MIN_ANGLE_RAD = Units.degreesToRadians(TurretConstants.MIN_ANGLE);
    private static final double MAX_ANGLE_RAD = Units.degreesToRadians(TurretConstants.MAX_ANGLE);

    private static double MAX_VEL_RAD_PER_SEC = 4*Units.degreesToRadians(360);
    private static double MAX_ACCEL_RAD_PER_SEC2 = 4*Units.degreesToRadians(720);

    private static final double VERSA_RATIO = 5.0;
    private static final double TURRET_RATIO = 140.0 / 10.0;
    private static final double GEAR_RATIO = VERSA_RATIO * TURRET_RATIO;

    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    /* ---------------- Hardware ---------------- */

    private final TalonFX motor;
    private TalonFXSimState simState;
    private SingleJointedArmSim turretSim;

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    /* ---------------- Profiling ---------------- */

    private TrapezoidProfile profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                MAX_VEL_RAD_PER_SEC,
                MAX_ACCEL_RAD_PER_SEC2));

    private State setpoint = new State();
    private Rotation2d goalAngle = Rotation2d.kZero;
    private double goalVelocityRadPerSec = 0.0;
    private double lastGoalRad = 0.0;

    /* ---------------- Visualization ---------------- */

    private final Mechanism2d mech = new Mechanism2d(100, 100);
    private final MechanismRoot2d root = mech.getRoot("turret", 50, 50);
    private final MechanismLigament2d ligament =
        root.append(new MechanismLigament2d("barrel", 30, 0));

    /* ---------------- Tuning ---------------- */

    private double kP = 15.0;
    private double kD = 0.0;

    /* ---------------- Constructor ---------------- */

    public Turret() {
        motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.RIO_CAN);
        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.Slot0.kP = kP;
        cfg.Slot0.kD = kD;

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity =
            Units.radiansToRotations(MAX_VEL_RAD_PER_SEC) * GEAR_RATIO;
        mm.MotionMagicAcceleration =
            Units.radiansToRotations(MAX_ACCEL_RAD_PER_SEC2) * GEAR_RATIO;

        motor.getConfigurator().apply(cfg);

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

    public double getPositionDeg() {
        return inputs.positionDeg;
    }

    public double getSetpointDeg(){
        return Units.radiansToDegrees(setpoint.position);
    }

    @Override
    public void updateInputs(){
        inputs.motorCurrent = motor.getTorqueCurrent().getValueAsDouble();
        inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / GEAR_RATIO;
        inputs.velocity = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble()) * TurretConstants.TURRET_RADIUS / GEAR_RATIO;
    }

    /* ---------------- Periodic ---------------- */

    @Override
    public void periodic() {
        updateInputs();

        double robotRelativeGoal = goalAngle.getRadians();

        // MA-style continuous optimization
        double best = lastGoalRad;
        boolean found = false;

        for (int i = -2; i <= 2; i++) {
            double candidate = robotRelativeGoal + Math.PI * 2 * i;
            if (candidate < MIN_ANGLE_RAD || candidate > MAX_ANGLE_RAD) continue;

            if (!found || Math.abs(candidate - lastGoalRad) < Math.abs(best - lastGoalRad)) {
                best = candidate;
                found = true;
            }
        }

        lastGoalRad = best;

        State goal =
            new State(
                MathUtil.clamp(best, MIN_ANGLE_RAD, MAX_ANGLE_RAD),
                goalVelocityRadPerSec);

        setpoint =
            profile.calculate(
                Constants.LOOP_TIME,
                setpoint,
                goal);

        double motorRot =
            Units.radiansToRotations(setpoint.position) * GEAR_RATIO;

        motor.setControl(mmRequest.withPosition(motorRot));

        ligament.setAngle(getPositionDeg());

        SmartDashboard.putNumber("Turret Pos Deg", getPositionDeg());
        SmartDashboard.putNumber("Turret Goal Deg", Units.radiansToDegrees(best));
    }

    /* ---------------- Simulation ---------------- */

    @Override
    public void simulationPeriodic() {
        turretSim.setInputVoltage(motor.getMotorVoltage().getValueAsDouble());
        turretSim.update(Constants.LOOP_TIME);

        double motorRot =
            Units.radiansToRotations(turretSim.getAngleRads()) * GEAR_RATIO;

        simState.setRawRotorPosition(motorRot);
        simState.setRotorVelocity(
            Units.radiansToRotations(turretSim.getVelocityRadPerSec()) * GEAR_RATIO);
    }
}
