package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;
    public double leftCurrent = 0.0;
    public double rightCurrent = 0.0;
    public double rollerVelocity = 0.0;
    public double rollerCurrent = 0.0;
    public double rightVoltage = 0.0;
    public double leftVoltage = 0.0;
    public double rollerSetSpeed = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs);

  /**
   * Get the intake extender position
   * 
   * @return inches
   */
  double getPosition();

  /**
   * Set the intake extender position
   * 
   * @param setpoint in inches
   */
  void setPosition(double setpoint);

  void setLeftMotor(double speed);

  void setRightMotor(double speed);

  void setRollerMotor(double speed);

  void setLimits(CurrentLimitsConfigs limits);

  void setRawPosition(double pos);

  /**
   * Reclaim all the resources (e.g., motors and other devices).
   * This step is necessary for multiple unit tests to work.
   */
  void close();
}
