package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodInputsIO{
        double measuredAngle = 0.0;
    }

    public void updateInputs();
}
