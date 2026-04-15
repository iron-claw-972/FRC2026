package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class ShooterIOTalonFX implements ShooterIO {

  private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.CANIVORE_SUB);
  private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.CANIVORE_SUB);

  // TODO Add current limits

  // Velocity in rotations per second
  VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public ShooterIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.5; // 0.5 stable
    config.Slot0.kI = 0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.125; // Maximum rps = 100 --> 12V/100rps

    config.CurrentLimits
        .withSupplyCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);

    shooterMotorLeft.getConfigurator().apply(config);
    shooterMotorRight.getConfigurator().apply(config);

    shooterMotorLeft.getConfigurator().apply(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast));

    shooterMotorRight.getConfigurator().apply(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    //TODO add higher stator current lim
    limitConfig.StatorCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;
    limitConfig.StatorCurrentLimitEnable = true;
    shooterMotorLeft.getConfigurator().apply(limitConfig);
    shooterMotorRight.getConfigurator().apply(limitConfig);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterSpeedLeft = Units.rotationsToRadians(shooterMotorLeft.getVelocity().getValueAsDouble())
        * ShooterConstants.SHOOTER_LAUNCH_DIAMETER / 2;
    inputs.shooterSpeedRight = Units.rotationsToRadians(shooterMotorRight.getVelocity().getValueAsDouble())
        * ShooterConstants.SHOOTER_LAUNCH_DIAMETER / 2;
    inputs.shooterCurrentLeft = shooterMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.shooterCurrentRight = shooterMotorRight.getStatorCurrent().getValueAsDouble();

  }

  @Override
  public void setNewCurrentLimit(double newCurrentLimit) {
    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    limitConfig.StatorCurrentLimit = newCurrentLimit;
    limitConfig.StatorCurrentLimitEnable = true;
    shooterMotorLeft.getConfigurator().apply(limitConfig);
    shooterMotorRight.getConfigurator().apply(limitConfig);
  }

  @Override
  public void setTargetVelocityRps(double target) {
        shooterMotorLeft.setControl(voltageRequest.withVelocity(target).withEnableFOC(true));
        shooterMotorRight.setControl(voltageRequest.withVelocity(target).withEnableFOC(true));
  }
}
