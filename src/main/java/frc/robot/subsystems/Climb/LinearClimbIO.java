package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface LinearClimbIO {
    @AutoLog
    public static class LinearClimbIOInputs{
        public double positionMeters = 0.0;
        public double motorCurrent = 0.0;
    }

    public void updateInputs();
}
