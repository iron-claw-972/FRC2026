package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double positionDeg = 0;
        public double velocity = 0;
        public double motorCurrent = 0;
    }

    public void updateInputs();
}
