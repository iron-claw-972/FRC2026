package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double positionDeg = 0;
    public double velocityRadPerSec = 0;
    public double motorStatorCurrent = 0;
    public double motorSupplyCurrent = 0;
    public double encoderLeftRot = 0;
    public double encoderRightRot = 0;
    public double motorVoltage = 0;
  }

  public void updateInputs(TurretIOInputs inputs);

  public void setMotorRaw(double speed);

  public void setControl(ControlRequest request);

  /**
   * sets supply and stator current limits
   * 
   * @param limit the current limit for stator and supply current
   */
  public void setCurrentLimits(double limit);
}
