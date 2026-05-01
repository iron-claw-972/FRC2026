package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double spindexerOneVelocity = 0.0;
        public double spindexerOneSupplyCurrent = 0.0;
        public double spindexerOneStatorCurrent = 0.0;
        public double spindexerTwoVelocity = 0.0;
        public double spindexerTwoStatorCurrent = 0.0;
        public double spindexerTwoSupplyCurrent = 0.0;
    }

  public void updateInputs(SpindexerIOInputs inputs);

  public void setControl(ControlRequest request);

  public void setPositionRaw(double pos);

  public void setNewCurrentLimit(double newCurrentLimit);
}
