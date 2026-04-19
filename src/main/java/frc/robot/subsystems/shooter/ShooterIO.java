package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterSpeedLeft = 0.0;
        public double shooterSpeedRight = 0.0;
        public double shooterStatorCurrentLeft = 0.0;
        public double shooterStatorCurrentRight = 0.0;
        public double shooterSupplyCurrentLeft = 0.0;
        public double shooterSupplyCurrentRight = 0.0;
    }

    public void updateInputs();
}
