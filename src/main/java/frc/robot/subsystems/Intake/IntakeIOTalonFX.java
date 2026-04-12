package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {

  // create the motors
  /** Motor to move the roller */
  private TalonFX rollerMotor = new TalonFX(IdConstants.ROLLER_MOTOR_ID, Constants.CANIVORE_SUB);
  /** Right motor (master) */
  private TalonFX rightMotor = new TalonFX(IdConstants.RIGHT_MOTOR_ID, Constants.CANIVORE_SUB);
  /** Left motor (slave) */
  private TalonFX leftMotor = new TalonFX(IdConstants.LEFT_MOTOR_ID, Constants.CANIVORE_SUB);

  private final MotionMagicVoltage voltageRequest = new MotionMagicVoltage(0);

  /**
   * Motor characteristics for the roller motor, a single Kraken X44 (aka gearbox)
   */
  private final DCMotor dcMotorRoller = DCMotor.getKrakenX44(1);
  /**
   * Motor characteristics for the extending pair of Kraken X44 motors (aka
   * gearbox)
   */
  private final DCMotor dcMotorExtend = DCMotor.getKrakenX44(2);

  public IntakeIOTalonFX() {

    // get the maximum free speed
    double maxFreeSpeed = Units.radiansToRotations(dcMotorExtend.freeSpeedRadPerSec) / IntakeConstants.GEAR_RATIO;

    // max free speed (rot/s) = motor free speed (rad/s to rot/s)/ gear ratio
    // safety margin, limits velocity to .75 free speed
    double maxVelocity = 0.75 * maxFreeSpeed;
    double maxAcceleration = maxVelocity / 0.25;

    // Configure the motors
    // Build the configuration for the roller
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

    // config Slot 0 PID params
    var slot0Configs = rollerConfig.Slot0;
    slot0Configs.kP = 5.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    slot0Configs.kV = 0.0;
    slot0Configs.kA = 0.0;

    // set the brake mode
    rollerConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    // apply the configuration to the right motor (master)
    rollerMotor.getConfigurator().apply(rollerConfig);

    // Build the configuration for the left and right Motor
    TalonFXConfiguration config = new TalonFXConfiguration();

    // config the current limits (low value for testing)
    config.CurrentLimits
        .withStatorCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMITS)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMITS)
        .withSupplyCurrentLimitEnable(true);

    // config Slot 0 PID params
    var rollerSlot0Configs = config.Slot0;
    rollerSlot0Configs.kP = 5.0;
    rollerSlot0Configs.kI = 0.0;
    rollerSlot0Configs.kD = 0.0;
    rollerSlot0Configs.kV = 0.0;
    rollerSlot0Configs.kA = 0.0;

    // configure MotionMagic
    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.GEAR_RATIO * maxVelocity
        / IntakeConstants.RADIUS_RACK_PINION / Math.PI / 2;
    motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.GEAR_RATIO * maxAcceleration
        / IntakeConstants.RADIUS_RACK_PINION / Math.PI / 2;

    rightMotor.getConfigurator().apply(config);
    leftMotor.getConfigurator().apply(config);

    leftMotor.getConfigurator().apply(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast));

    rightMotor.getConfigurator().apply(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
    limitConfig.StatorCurrentLimit = IntakeConstants.NORMAL_CURRENT_LIMIT;
    limitConfig.StatorCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(limitConfig);
    rightMotor.getConfigurator().apply(limitConfig);

    leftMotor.setPosition(0.0);
    rightMotor.setPosition(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftPosition = Intake.rotationsToInches(leftMotor.getPosition().getValueAsDouble());
    inputs.rightPosition = Intake.rotationsToInches(rightMotor.getPosition().getValueAsDouble());
    inputs.leftCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
    inputs.rollerCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerSetSpeed = rollerMotor.get();
  }

  /**
   * Set the intake extender position
   * 
   * @param setpoint in inches
   */
  @Override
  public void setPosition(double setpoint) {
    double motorRotations = -Intake.inchesToRotations(setpoint);
    rightMotor.setControl(voltageRequest.withPosition(motorRotations).withEnableFOC(true));
    leftMotor.setControl(voltageRequest.withPosition(motorRotations).withEnableFOC(true));
  }

  /**
   * Get the intake extender position
   * 
   * @return inches
   */
  @Override
  public double getPosition() {
    return Intake.rotationsToInches(leftMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setLeftMotor(double speed) {
    leftMotor.set(speed);
  }

  @Override
  public void setRightMotor(double speed) {
    rightMotor.set(speed);
  }

  @Override
  public void setRollerMotor(double speed) {
    rollerMotor.set(speed);
  }

  @Override
  public void setLimits(CurrentLimitsConfigs limits) {
    leftMotor.getConfigurator().apply(limits);
    rightMotor.getConfigurator().apply(limits);
  }

  @Override
  public void setRawPosition(double pos) {
    leftMotor.setPosition(pos);
    rightMotor.setPosition(pos);
  }

  
  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
    rollerMotor.close();
  }

}
