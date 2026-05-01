package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class TurretIOTalonFX implements TurretIO {

  private final TalonFX motor = new TalonFX(IdConstants.TURRET_MOTOR_ID, Constants.CANIVORE_SUB);

  public TurretIOTalonFX() {
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = 12.0;
    config.Slot0.kS = 0.1; // Static friction compensation
    config.Slot0.kV = 0.12; // Adjusted kV for the gear ratio
    config.Slot0.kD = 0.0; // The "Braking" term to stop overshoot

    CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
    limitsConfigs.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
    limitsConfigs.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
    motor.getConfigurator().apply(limitsConfigs);

    var mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity = Units.radiansToRotations(TurretConstants.MAX_VELOCITY) * TurretConstants.GEAR_RATIO;
    mm.MotionMagicAcceleration = Units.radiansToRotations(TurretConstants.MAX_ACCELERATION)
        * TurretConstants.GEAR_RATIO; // Lowered for belt safety
    mm.MotionMagicJerk = 0; // Set to > 0 for "S-Curve" smoothing if needed
    motor.getConfigurator().apply(config);

    motor.setPosition(0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionDeg = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / TurretConstants.GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble())
        / TurretConstants.GEAR_RATIO;
    inputs.motorStatorCurrent = motor.getStatorCurrent().getValueAsDouble();
    inputs.motorSupplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();

  }

  @Override
  public void setMotorRaw(double speed) {
    motor.set(speed);
  }

  @Override
  public void setControl(ControlRequest request) {
    motor.setControl(request);
  }

  /**
   * sets supply and stator current limits
   * 
   * @param limit the current limit for stator and supply current
   */
  @Override
  public void setCurrentLimits(double limit) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(limit)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);

    if (limit == TurretConstants.SUPPLY_CURRENT_LIMIT) {
      limits.SupplyCurrentLowerLimit = TurretConstants.SUPPLY_CURRENT_LIMIT * .5;
      limits.SupplyCurrentLowerTime = 1.0; // Set to lower limit if over 1 second
    }

    motor.getConfigurator().apply(limits);

  }

}
