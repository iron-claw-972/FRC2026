package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double leftPosition = 0.0;
        public double rightPosition = 0.0;
        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerCurrent = 0.0;
    }

    public void updateInputs();
}
