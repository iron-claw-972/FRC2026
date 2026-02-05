package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;

public class LinearClimb {
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private PIDController pid = new PIDController(0.01, 0, 0);

    public LinearClimb() {

    }

    public void periodic() {

    }

    public void setSetpoint(double setpoint) {

    }

    public void climb() {
        
    }
}