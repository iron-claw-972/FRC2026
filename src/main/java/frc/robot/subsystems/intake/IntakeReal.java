package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IdConstants;

public class IntakeReal extends IntakeBase {
    private TalonFX flyWheelMotor;
    private TalonFX baseMotor;
    private double position;

    private double baseVelocity;
    private double flyWheelVelocity;
    
    double basePower;
    double flyWheelPower;

    private PIDController pid = new PIDController(0.2, 0.0, 0.05);
    private double intakeGearRatio = 67.0/67.0;

    private SingleJointedArmSim intakeSim;
    private static final DCMotor baseIntakeMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("baseMotor", 25, 0));

    public IntakeReal() {
        baseMotor = new TalonFX(IdConstants.BASE_MOTOR_ID);
        flyWheelMotor = new TalonFX(IdConstants.FLYWHEEL_MOTOR_ID);

        pid.setTolerance(Units.degreesToRadians(3));
        encoderSim = baseMotor.getSimState();

        intakeSim = new SingleJointedArmSim(
            baseIntakeMotorSim,
            intakeGearRatio,
            HoodConstants.MOI,
            HoodConstants.LENGTH,
            0,
            Units.degreesToRadians(360),
            false,
            Units.degreesToRadians(HoodConstants.START_ANGLE)
        );

        baseMotor.setPosition(Units.degreesToRadians(HoodConstants.START_ANGLE * intakeGearRatio));
        baseMotor.setNeutralMode(NeutralModeValue.Brake);

        flyWheelMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        SmartDashboard.putData("intake", mechanism2d);
        SmartDashboard.putData("PID", pid);
        
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setSetpoint(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setSetpoint(270)));    
    }

    @Override
    public void setSetpoint(double setPoint) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(setPoint));
    }

    @Override
    public double getPosition() {
        return position;
    }
    
    @Override
    public double getBaseMotorVelocity() {
        return baseVelocity/intakeGearRatio;
    }

    @Override
    public double getFlyWheelVelocity() {
        return flyWheelVelocity;
    }

    @Override
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        position = Units.rotationsToDegrees(baseMotor.getPosition().getValueAsDouble());
        baseVelocity = Units.rotationsPerMinuteToRadiansPerSecond(baseMotor.getVelocity().getValueAsDouble() * 60);
        flyWheelVelocity = Units.rotationsPerMinuteToRadiansPerSecond(flyWheelMotor.getVelocity().getValueAsDouble() * 60);

        basePower = pid.calculate(Units.degreesToRadians(getPosition()));
        baseMotor.set(basePower);

        flyWheelMotor.set(flyWheelPower);

        ligament2d.setAngle(position);
    }

    public double getAppliedVoltage() {
        return baseMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        double voltsMotor = basePower * 12;
        intakeSim.setInputVoltage(voltsMotor);

        intakeSim.update(Constants.LOOP_TIME);

        double simAngle = intakeSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * intakeGearRatio;

        encoderSim.setRawRotorPosition(motorRotations);
    }

    @Override
    public boolean flyWheelSpinning() {
        return flyWheelMotor.getVelocity().getValueAsDouble() > 0;
    }

    @Override
    public void setFlyWheel(double speed) {
        flyWheelMotor.set(speed);
    } 
}