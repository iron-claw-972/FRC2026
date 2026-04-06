package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double spindexerVelocityOne = 0.0;
        public double spindexerCurrentOne = 0.0;
        public double spindexerVelocityTwo = 0.0;
        public double spindexerCurrentTwo = 0.0;
    }

    public void updateInputs();
}
