package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class IntakeAlpha extends SubsystemBase{
    private TalonFX motor = new TalonFX(IdConstants.FLYWHEEL_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private double power = 0;
    private double x = 1;

    @Override
    public void periodic(){
        x = SmartDashboard.getNumber("intake speed", x);
        SmartDashboard.putNumber("intake speed", x);
        motor.set(-power * x);
    }

    public void setPower(double power){
        this.power = power;
    }
}
