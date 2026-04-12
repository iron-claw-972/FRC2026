package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.IdConstants;
import sun.jvm.hotspot.utilities.Unsigned5.SetPosition;

public class HoodIOTalonFX implements HoodIO {
  private TalonFX motor = new TalonFX(IdConstants.HOOD_ID, Constants.CANIVORE_SUB);

  public HoodIOTalonFX() {
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = 2.0;
    config.Slot0.kS = 0.1; // Static friction compensation
    config.Slot0.kV = 0.12; // Adjusted kV for the gear ratio
    config.Slot0.kD = 0.02; // The "Braking" term to stop overshoot

    var mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity = Units.radiansToRotations(HoodConstants.MAX_VELOCITY) * HoodConstants.HOOD_GEAR_RATIO;
    mm.MotionMagicAcceleration = Units.radiansToRotations(HoodConstants.MAX_ACCELERATION)
        * HoodConstants.HOOD_GEAR_RATIO; // Lowered for belt safety
    mm.MotionMagicJerk = 0; // Set to > 0 for "S-Curve" smoothing if needed
    motor.getConfigurator().apply(config);

    setCurrentLimits(HoodConstants.NORMAL_CURRENT_LIMIT);

    motor.setPosition(Units.degreesToRotations(HoodConstants.MAX_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble())
        / HoodConstants.HOOD_GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble())
        / HoodConstants.HOOD_GEAR_RATIO;
    inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setMotorRaw(double speed) {
    motor.set(speed);

  }

  @Override
  public void setMotorControl(MotionMagicVoltage control) {
    motor.setControl(control);
  }

  @Override
  public void setPositionRaw(double pos) {
    motor.setPosition(pos);
  }

  /**
   * sets supply and stator current limits
   * 
   * @param limitAmps the current limit for stator and supply current
   */
  @Override
  public void setCurrentLimits(double limitAmps) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(limitAmps)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limitAmps);

    motor.getConfigurator().apply(limits);
  }
}
