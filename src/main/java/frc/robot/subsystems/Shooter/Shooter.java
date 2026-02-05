package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    // rotations/sec

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;

    public boolean shooterAtMaxSpeed = false;
    public boolean ballDetected = false;
    // Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    double powerModifier;

    public Shooter() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1; // tune p value
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0.12; // Maximum rps = 100 --> 12V/100rps
        shooterMotorLeft.getConfigurator().apply(config);
        shooterMotorRight.getConfigurator().apply(config);

        shooterMotorLeft.getConfigurator().apply(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        shooterMotorRight.getConfigurator().apply(
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

        SmartDashboard.putData("shoot", new InstantCommand(() -> setShooter(10 * powerModifier)));

    }

    @Override
    public void periodic() {
        powerModifier = SmartDashboard.getNumber("shooter power modifier", powerModifier);
        SmartDashboard.putNumber("shooter power modifier", powerModifier);

        shooterMotorLeft.setControl(voltageRequest.withVelocity(-shooterTargetSpeed * powerModifier));
        shooterMotorRight.setControl(voltageRequest.withVelocity(-shooterTargetSpeed * powerModifier));
    }

    public void deactivateShooter() {
        setShooter(0);
    }

    public void setShooter(double linearVelocityMps) {
        double wheelCircumference = Math.PI * ShooterConstants.SHOOTER_WHEEL_DIAMETER;
        shooterTargetSpeed = linearVelocityMps * ShooterConstants.SHOOTER_GEAR_RATIO / wheelCircumference; // rps
    }

    /** @return velocity in m/s */
    public double getShooterVelcoity() {
        return Units.rotationsToRadians(shooterMotorLeft.getVelocity().getValueAsDouble())
                * ShooterConstants.SHOOTER_WHEEL_DIAMETER / 2;
    }

    public boolean atTargetSpeed() {
        return Math.abs(getShooterVelcoity() - shooterTargetSpeed) < 1.0;
    }

    @AutoLogOutput(key = "Shooter/TargetSpeed")
    public double getTargetVelocity() {
        return Units.rotationsToRadians(shooterTargetSpeed) * ShooterConstants.SHOOTER_WHEEL_DIAMETER / 2;
    }
}
