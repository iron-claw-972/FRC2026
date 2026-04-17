package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Spindexer extends SubsystemBase implements SpindexerIO {
    private TalonFX motorOne = new TalonFX(IdConstants.SPINDEXER_ONE_ID, Constants.CANIVORE_SUB);
    private TalonFX motorTwo = new TalonFX(IdConstants.SPINDEXER_TWO_ID, Constants.CANIVORE_SUB);

    private double power = 0.0;
    public int ballCount = 0;
    private boolean wasSpindexerSlow = false;
    private SpindexerState state = SpindexerState.STOPPED;
    private boolean reversing = false;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public boolean noIndexing = false;


    public Spindexer() {
        updateInputs();

        // configure current limit
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = SpindexerConstants.CURRENT_SPIKE_LIMIT;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLowerLimit = SpindexerConstants.currentLimit;
        limitConfig.SupplyCurrentLowerTime = 1.5;
        motorOne.getConfigurator().apply(limitConfig);
        motorTwo.getConfigurator().apply(limitConfig);

        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putData("Spindexer Run Forward", new InstantCommand(() -> maxSpindexer()));
            SmartDashboard.putData("Spindexer Run Reverse", new InstantCommand(() -> reverseSpindexer()));
            SmartDashboard.putData("Spindexer Stop", new InstantCommand(() -> stopSpindexer()));
        }
    }

    public enum SpindexerState {
        MAX,
        REVERSE,
        STOPPED,
        CUSTOM,
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Spindexer", inputs);

        if (state == SpindexerState.MAX) {
            setMotorVoltages(SpindexerConstants.spindexerForwardVoltage);
            reversing = false;
        } else if (state == SpindexerState.REVERSE) {
            setMotorVoltages(SpindexerConstants.spindexerReverseVoltage);
            reversing = true;
        } else if (state == SpindexerState.STOPPED) {
            setMotorVoltages(0.0);
            reversing = false;
        } else {
            setMotorVoltages(power);
            reversing = false;
        }


        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Spindexer Running", state == SpindexerState.MAX || state == SpindexerState.CUSTOM);
            SmartDashboard.putBoolean("Spindexer Has Ball", ballCount > 0);
        }

        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Spindexer Reversing", state == SpindexerState.REVERSE);
        }
    }

    public void setMotorVoltages(double voltage) {
        motorOne.setControl(new VoltageOut(voltage).withEnableFOC(true));
        motorTwo.setControl(new VoltageOut(voltage).withEnableFOC(true));
    }

    public void maxSpindexer() {
        state = SpindexerState.MAX;
    }

    public void reverseSpindexer(){
        state = SpindexerState.REVERSE;
    }

    public void stopSpindexer() {
        state = SpindexerState.STOPPED;
    }

    public void setSpindexer(double power) {
        this.power = power;
        state = SpindexerState.CUSTOM;
    }

    public double getStatorCurrent() {
        return inputs.spindexerOneCurrent + inputs.spindexerTwoCurrent;
    }

    public void setNewCurrentLimit(double newCurrentLimit) {
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = newCurrentLimit;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLowerLimit = newCurrentLimit;
        limitConfig.SupplyCurrentLowerTime = 1.5;
        motorOne.getConfigurator().apply(limitConfig);
        motorTwo.getConfigurator().apply(limitConfig);
    }

    @Override
    public void updateInputs() {
        inputs.spindexerOneVelocity = motorOne.getVelocity().getValueAsDouble();
        inputs.spindexerOneCurrent = motorOne.getStatorCurrent().getValueAsDouble();
        inputs.spindexerTwoVelocity = motorTwo.getVelocity().getValueAsDouble();
        inputs.spindexerTwoCurrent = motorTwo.getStatorCurrent().getValueAsDouble();
    }
}
