package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double spindexerOneVelocity = 0.0;
        public double spindexerOneCurrent = 0.0;
        public double spindexerTwoVelocity = 0.0;
        public double spindexerTwoCurrent = 0.0;
    }

  public void updateInputs(SpindexerIOInputs inputs);

  public void setControl(ControlRequest request);

  public void setPositionRaw(double pos);

  public void setNewCurrentLimit(double newCurrentLimit);
}
