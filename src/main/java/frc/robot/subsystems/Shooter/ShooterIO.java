package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs{
        public double shooterSpeedLeft = 0.0;
        public double shooterSpeedRight = 0.0;
        public double feederSpeed = 0.0;
        public double sensorDistance = 0.0;
    }

    public void updateInputs();
}
