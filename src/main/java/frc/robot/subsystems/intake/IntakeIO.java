package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double measuredAngle = IntakeConstants.START_ANGLE;
        public double currentAmps = 0.0;
        public double flyWheelVelocity = 0.0;

    }

    public void updateInputs();
}
