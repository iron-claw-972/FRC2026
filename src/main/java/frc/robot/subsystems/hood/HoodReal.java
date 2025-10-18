package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.TalonFX;
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

public class HoodReal extends HoodBase {
    private TalonFX motor;
    private double position;
    private double velocity;
    double power;
    private PIDController pid = new PIDController(0.2, 0.0, 0.05);
    private double hoodGearRatio = 67.0/67.0;

    private SingleJointedArmSim hoodSim;
    private static final DCMotor hoodMotorSim = DCMotor.getKrakenX60(1);
    private TalonFXSimState encoderSim;

    Mechanism2d mechanism2d = new Mechanism2d(100, 100);
    MechanismRoot2d mechanismRoot = mechanism2d.getRoot("pivot", 50, 50);
    MechanismLigament2d ligament2d = mechanismRoot.append(new MechanismLigament2d("hoodMotor", 25, 0));

    public HoodReal() {
        motor = new TalonFX(IdConstants.HOOD_MOTOR_ID);
        pid.setTolerance(Units.degreesToRadians(2));
        encoderSim = motor.getSimState();

        hoodSim = new SingleJointedArmSim(
            hoodMotorSim,
            hoodGearRatio,
            HoodConstants.MOI,
            HoodConstants.LENGTH,
            0,
            Units.degreesToRadians(360),
            true,
            Units.degreesToRadians(HoodConstants.START_ANGLE)
        );

        SmartDashboard.putData("hood", mechanism2d);
        SmartDashboard.putData("PID", pid);
        
        SmartDashboard.putData("Set 90 degrees", new InstantCommand(() -> setSetpoint(90)));
        SmartDashboard.putData("Set 180 degrees", new InstantCommand(() -> setSetpoint(180)));
        SmartDashboard.putData("Set 0 degrees", new InstantCommand(() -> setSetpoint(0)));
        SmartDashboard.putData("Set 270 degrees", new InstantCommand(() -> setSetpoint(270)));
    }

    public void setSetpoint(double setPoint) {
        pid.reset();
        pid.setSetpoint(Units.degreesToRadians(setPoint));
    }

    public double getPosition() {
        return position;
    }
    
    public double getVelocity() {
        return velocity/hoodGearRatio;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
        power = pid.calculate(Units.degreesToRadians(getPosition()));
        motor.set(power);

        ligament2d.setAngle(position);
    }

    public double getAppliedVoltage() {
        
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        double voltsMotor = power * 12;
        hoodSim.setInputVoltage(voltsMotor);

        hoodSim.update(Constants.LOOP_TIME);

        double simAngle = hoodSim.getAngleRads();
        double simRotations = Units.radiansToRotations(simAngle);
        double motorRotations = simRotations * hoodGearRatio;

        encoderSim.setRawRotorPosition(motorRotations);
    }
}