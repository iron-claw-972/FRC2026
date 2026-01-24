package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double rightShooterVelocity = 0;
        public double leftShooterVelocity = 0;
        public double feederVelocity = 0;
        public double rightShooterCurrent = 0;
        public double leftShooterCurrent = 0;
        public double feederCurrent = 0;
    }

    public void updateInputs();
}
