package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Climb.ClimbConstants;

public class LinearClimb {
    private TalonFX motor;

    private double kP = 0;
    private double kI = 0.0;
    private double kD = 0.0;  
    
    private double gearRatio = ClimbConstants.CLIMB_GEAR_RATIO;

    public LinearClimb() {
        TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
            config.Slot0.kG = 0.0; // Gravity compensation
            config.Slot0.kV = 0.0; // Velocity gain: 1 rps -> 0.12V
            config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)
        
            config.Slot0.kP = kP; // If position error is 1 rotation, apply 10V
            config.Slot0.kI = kI; // Integral term (usually left at 0 for MotionMagic)
            config.Slot0.kD = kD; // Derivative term (used to dampen oscillations)
        
            MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = Units.radiansToRotations(ClimbConstants.MAX_VELOCITY) * gearRatio; // max velocity * gear ratio
            motionMagicConfigs.MotionMagicAcceleration = Units.radiansToRotations(ClimbConstants.MAX_ACCELERATION) * gearRatio; // max Acceleration * gear ratio

            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            motor.getConfigurator().apply(config);
    }

    public void periodic() {

    }

    public void setSetpoint(double setpoint) {

    }

    public void climb() {

    }
}