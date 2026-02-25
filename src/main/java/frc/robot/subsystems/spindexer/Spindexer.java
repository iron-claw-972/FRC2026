package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Spindexer extends SubsystemBase implements SpindexerIO {
    private TalonFX motor = new TalonFX(IdConstants.SPINDEXER_ID, Constants.CANIVORE_SUB);

    private double power = 0.0;
    public int ballCount = 0;
    private boolean wasSpindexerSlow = false;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer() {
        updateInputs();

        // configure current limit
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = SpindexerConstants.CURRENT_SPIKE_LIMIT;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLowerLimit = SpindexerConstants.currentLimit;
        limitConfig.SupplyCurrentLowerTime = 1.5;
        motor.getConfigurator().apply(limitConfig);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Spindexer", inputs);

        motor.set(power);
        
        // scale threshold based on power
        double velocityThreshold = SpindexerConstants.spindexerVelocityWithBall * power;
        SmartDashboard.putNumber("Spindexer Ball Count", ballCount);

        boolean isSpindexerSlow = inputs.spindexerVelocity < velocityThreshold;
        if (wasSpindexerSlow && !isSpindexerSlow && power > 0.1) {
            ballCount++;
        }
        wasSpindexerSlow = isSpindexerSlow;
    }

    public void maxSpindexer() {
        power = SpindexerConstants.spindexerMaxPower;
    }

    public void reverseSpindexer(){
        power = SpindexerConstants.spindexerReversePower;
    }

    public void stopSpindexer() {
        power = 0.0;
    }

    public void setSpindexer(double power) {
        this.power = power;
    }

    @Override
    public void updateInputs() {
        inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble(); //SpindexerConstants.gearRatio;
        inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
        Logger.processInputs("Spindexer", inputs);
    }

}
