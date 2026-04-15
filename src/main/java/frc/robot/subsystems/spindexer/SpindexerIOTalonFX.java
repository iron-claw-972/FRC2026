package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class SpindexerIOTalonFX implements SpindexerIO {

  private TalonFX motor = new TalonFX(IdConstants.SPINDEXER_ID, Constants.CANIVORE_SUB);

  public SpindexerIOTalonFX() {
    // configure current limit
    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    limitConfig.StatorCurrentLimit = SpindexerConstants.CURRENT_SPIKE_LIMIT;
    limitConfig.StatorCurrentLimitEnable = true;
    limitConfig.SupplyCurrentLowerLimit = SpindexerConstants.currentLimit;
    limitConfig.SupplyCurrentLowerTime = .2;
    motor.getConfigurator().apply(limitConfig);

  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble(); // SpindexerConstants.gearRatio;
    inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.spindexerPosition = motor.getPosition().getValueAsDouble();
  }

  @Override
  public void setNewCurrentLimit(double newCurrentLimit) {
    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    limitConfig.StatorCurrentLimit = newCurrentLimit;
    limitConfig.StatorCurrentLimitEnable = true;
    limitConfig.SupplyCurrentLowerLimit = newCurrentLimit;
    limitConfig.SupplyCurrentLowerTime = 1.5;
    motor.getConfigurator().apply(limitConfig);
  }

  @Override
  public void setControl(ControlRequest request) {
    motor.setControl(request);
  }

  @Override
  public void setPositionRaw(double pos) {
    motor.setPosition(pos);
  }

}
