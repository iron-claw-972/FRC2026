package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double spindexerVelocity = 0.0;
        public double spindexerCurrent = 0.0;
    }

    public void updateInputs();
}
