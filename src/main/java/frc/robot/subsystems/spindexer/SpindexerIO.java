package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double spindexerOneVelocity = 0.0;
        public double spindexerOneCurrent = 0.0;
        public double spindexerTwoVelocity = 0.0;
        public double spindexerTwoCurrent = 0.0;
    }

    public void updateInputs();
}
