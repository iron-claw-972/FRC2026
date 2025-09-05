package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class TurretBase extends SubsystemBase {
    double position;
    private TalonFX turrotBaseMotor;
    TurretBase(double positon) {
        this.turrotBaseMotor = new TalonFX(IdConstants.Base_ID, "rio");

        this.turrotBaseMotor.setNeutralMode(NeutralModeValue.Coast);
        this.turrotBaseMotor.set(0);
    }
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

