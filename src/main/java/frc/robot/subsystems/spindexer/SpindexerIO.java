package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double spindexerVelocity = 0.0;
    public double spindexerCurrent = 0.0;
    public double spindexerPosition = 0.0;
  }

  public void updateInputs(SpindexerIOInputs inputs);

  public void setControl(ControlRequest request);

  public void setPositionRaw(double pos);

  public void setNewCurrentLimit(double newCurrentLimit);
}
