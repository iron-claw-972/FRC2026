package frc.robot.subsystems.Turret;

public class TurretBase {
    public void SetMotor(TalonFX motor, double speed){
        motor.set(speed);
    }
    public void StopMotor(TalonFX motor){
        SetMotor(motor, 0);
    }
    public double GetPosition(TalonFX motor){
        return motor.getPosition().getValue();
    }
}
