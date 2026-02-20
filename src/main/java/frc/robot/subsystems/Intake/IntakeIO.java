package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public static double leftPosition = 0.0;
        public static double rightPosition = 0.0;
        public static double leftCurrent = 0.0;
        public static double rightCurrent = 0.0;
    }

    public void updateInputs();
}
