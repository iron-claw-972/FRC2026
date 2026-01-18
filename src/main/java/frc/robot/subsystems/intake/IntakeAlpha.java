package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class IntakeAlpha extends SubsystemBase{
    private TalonFX motor = new TalonFX(IdConstants.FLYWHEEL_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private double power = 0;

    @Override
    public void periodic(){
        motor.set(power);
    }

    public void setPower(double power){
        this.power = power;
    }
}
