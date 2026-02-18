package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double positionDeg = HoodConstants.MAX_ANGLE;
        public double velocityRadPerSec = 0.0;
        public double motorCurrent = 0.0;
    }

    public void updateInputs();
}
