package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double leftPosition = 0.0;
        public double rightPosition = 0.0;
        public double leftSupplyCurrent = 0.0;
        public double rightSupplyCurrent = 0.0;
        public double leftStatorCurrent = 0.0;
        public double rightStatorCurrent = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerSupplyCurrent = 0.0;
        public double rollerStatorCurrent = 0.0;
    }

    public void updateInputs();
}
