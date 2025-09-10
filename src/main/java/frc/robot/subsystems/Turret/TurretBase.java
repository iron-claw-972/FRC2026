package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class TurretBase extends SubsystemBase {
    double position;
    private TalonFX turretBaseMotor;
    TurretBase(double positon) {
        this.turretBaseMotor = new TalonFX(IdConstants.Base_ID, "rio");

        this.turretBaseMotor.setNeutralMode(NeutralModeValue.Coast);
        this.turretBaseMotor.set(0);
    }
    public void SetMotor(TalonFX motor, double speed){
        motor.set(speed);
    }
    public void StopMotor(TalonFX motor){
        SetMotor(motor, 0);
    }
    public Angle GetPosition(TalonFX motor){
        return motor.getPosition().getValue();
    }
}

