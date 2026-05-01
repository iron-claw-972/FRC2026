package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface HoodIO {
        @AutoLog
        public static class HoodIOInputs {
                public double positionDeg = HoodConstants.MAX_ANGLE;
                public double velocityRadPerSec = 0.0;
                public double motorCurrent = 0.0;
                public double motorVoltage = 0.0;
        }

        public void updateInputs(HoodIOInputs inputs);

        public void setMotorRaw(double speed);

        public void setMotorControl(MotionMagicVoltage control);

        public void setPositionRaw(double pos);

        /**
         * sets supply and stator current limits
         * 
         * @param limitAmps the current limit for stator and supply current
         */
        public void setCurrentLimits(double stator, double supply);
}
