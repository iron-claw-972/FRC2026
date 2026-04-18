package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class SpindexerIOTalonFX implements SpindexerIO {

  private TalonFX motorOne = new TalonFX(IdConstants.SPINDEXER_ONE_ID, Constants.CANIVORE_SUB);
  private TalonFX motorTwo = new TalonFX(IdConstants.SPINDEXER_TWO_ID, Constants.CANIVORE_SUB);

  public SpindexerIOTalonFX() {
    // configure current limit
    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    limitConfig.StatorCurrentLimit = SpindexerConstants.CURRENT_SPIKE_LIMIT;
    limitConfig.StatorCurrentLimitEnable = true;
    limitConfig.SupplyCurrentLowerLimit = SpindexerConstants.currentLimit;
    limitConfig.SupplyCurrentLowerTime = 1.5;
    motorOne.getConfigurator().apply(limitConfig);
    motorTwo.getConfigurator().apply(limitConfig);

    // Invert motor two so they spin in opposite directions
    MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    motorConfig.Inverted = InvertedValue.Clockwise_Positive;
    motorTwo.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.spindexerOneVelocity = motorOne.getVelocity().getValueAsDouble();
    inputs.spindexerOneCurrent = motorOne.getStatorCurrent().getValueAsDouble();
    inputs.spindexerTwoVelocity = motorTwo.getVelocity().getValueAsDouble();
    inputs.spindexerTwoCurrent = motorTwo.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setNewCurrentLimit(double newCurrentLimit) {
    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    limitConfig.StatorCurrentLimit = newCurrentLimit;
    limitConfig.StatorCurrentLimitEnable = true;
    limitConfig.SupplyCurrentLowerLimit = newCurrentLimit;
    limitConfig.SupplyCurrentLowerTime = 1.5;
    motorOne.getConfigurator().apply(limitConfig);
    motorTwo.getConfigurator().apply(limitConfig);
  }

  @Override
  public void setControl(ControlRequest request) {
    motorOne.setControl(request);
    motorTwo.setControl(request);
  }

  @Override
  public void setPositionRaw(double pos) {
    motorOne.setPosition(pos);
    motorTwo.setPosition(pos);
  }
}
