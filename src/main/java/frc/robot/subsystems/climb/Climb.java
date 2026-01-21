package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase{
    private TalonFX ClimbMotor;
    private DutyCycleEncoder ClimbEncoder;
    private PIDController ClimbPID;

    public Climb(){
        ClimbMotor = new TalonFX(ClimbConstants.ClimbMotorID);
        ClimbEncoder = new DutyCycleEncoder(ClimbConstants.ClimbEncoderID);
        ClimbPID = new PIDController(1, 0, 0);

        ClimbMotor.setPosition(ClimbEncoder.get() * ClimbConstants.gear_ratio);
    }

    public void setSetpoint(double Setpoint){
        ClimbPID.setSetpoint(Setpoint * ClimbConstants.gear_ratio);
    }

    public boolean atSetpoint(){
        return ClimbPID.atSetpoint();
    }

    public void periodic(){
        ClimbMotor.set( ClimbPID.calculate(ClimbMotor.getPosition().getValueAsDouble()) + 0.0); //TODO add proper value
    }

    public void ClimbToFirstPosition(){
        setSetpoint(ClimbConstants.climbFirstStage * ClimbConstants.gear_ratio + ClimbMotor.getPosition().getValueAsDouble());
    }

    public void ClimbToSecondPosition(){
        setSetpoint(ClimbConstants.climbSecondStage * ClimbConstants.gear_ratio + ClimbMotor.getPosition().getValueAsDouble());
    }
}
