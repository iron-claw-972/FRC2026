package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Spindexer extends SubsystemBase implements SpindexerIO {
    private TalonFX motor = new TalonFX(IdConstants.SPINDEXER_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private double power = 0.0;
    public int ballCount = 0;
    private boolean wasSpindexerSlow = false;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer() {
        updateInputs();

        // configure current limit
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = SpindexerConstants.currentLimit;
        limitConfig.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(limitConfig);

        SmartDashboard.putData("Max speed spindexer", new InstantCommand(() -> maxSpindexer()));
        SmartDashboard.putData("Turn off spindexer", new InstantCommand(() -> stopSpindexer()));
        SmartDashboard.putData("Spindexer 50%", new InstantCommand(() -> setSpindexer(0.5)));
    }

    @Override
    public void periodic() {
        updateInputs();

        // if speed was set
        if (SmartDashboard.containsKey("Spindexer Power")) {
            double dashboardPower = SmartDashboard.getNumber("Spindexer Power", 0.0);
            if (dashboardPower != power) {
                power = dashboardPower;
            }
        }

        motor.set(power);

        SmartDashboard.putNumber("Spindexer Power", power);
        SmartDashboard.putNumber("Spindexer Velocity", inputs.spindexerVelocity);
        
        // scale threshold based on power
        double velocityThreshold = SpindexerConstants.spindexerVelocityWithBall * power;
        SmartDashboard.putNumber("Spindexer Velocity Threshold", velocityThreshold);
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

    public void stopSpindexer() {
        power = 0.0;
    }

    public void setSpindexer(double power) {
        this.power = power;
    }

    @Override
    public void updateInputs() {
        inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble() * SpindexerConstants.gearRatio;
        inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
        Logger.processInputs("Spindexer", inputs);
    }

}
