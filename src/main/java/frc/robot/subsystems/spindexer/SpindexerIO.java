package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

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

    public void updateInputs();
}
