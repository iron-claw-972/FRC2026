package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.subsystems.spindexer.SpindexerIO;

public class Spindexer extends SubsystemBase implements SpindexerIO{
    private TalonFX motor = new TalonFX(IdConstants.SPINDEXER_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private double power = 0.0;
    public int ballCount = 0;
    private boolean wasSpindexerSlow = false;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer(){
        updateInputs();
    }

    @Override
    public void periodic() {
        updateInputs();
        
        double dashboardPower = SmartDashboard.getNumber("Spindexer Power", -1.0);
        if (dashboardPower != -1.0) {
            power = dashboardPower;
        }
        
        motor.set(power);
        
        boolean isSpindexerSlow = inputs.spindexerVelocity < SpindexerConstants.spindexerVelocityWithBall;
        if (wasSpindexerSlow && !isSpindexerSlow && power > 0.1) {
            ballCount++;
        }
        wasSpindexerSlow = isSpindexerSlow;
    }

    public void maxSpindexer(){
        power = SpindexerConstants.spindexerMaxPower;
    }

    public void stopSpindexer(){
        power = 0.0;
    }

    @Override
    public void updateInputs() {
        inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble();
        inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
        Logger.processInputs("Spindexer", inputs);
    }

}
