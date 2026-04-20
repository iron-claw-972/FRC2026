package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import org.littletonrobotics.junction.Logger;

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
    private SpindexerState state = SpindexerState.STOPPED;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public boolean noIndexing = false;


    public Spindexer() {
        updateInputs();

        // configure current limit
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = SpindexerConstants.SUPPLY_CURRENT_LIMIT;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLowerLimit = SpindexerConstants.CURRENT_FORWARD_STATOR_LIMIT;
        limitConfig.SupplyCurrentLimitEnable = true;
        motorOne.getConfigurator().apply(limitConfig);
        motorTwo.getConfigurator().apply(limitConfig);
        motorTwo.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

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
        } else if (state == SpindexerState.REVERSE) {
            setMotorVoltages(SpindexerConstants.spindexerReverseVoltage);
        } else if (state == SpindexerState.STOPPED) {
            setMotorVoltages(0.0);
        } else {
            setMotorVoltages(power);
        }

        if (state == SpindexerState.REVERSE) {
            setNewCurrentLimit(SpindexerConstants.SUPPLY_CURRENT_LIMIT, SpindexerConstants.CURRENT_REVERSE_STATOR_LIMIT);
        } else {
            setNewCurrentLimit(SpindexerConstants.SUPPLY_CURRENT_LIMIT, SpindexerConstants.CURRENT_FORWARD_STATOR_LIMIT);
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
        motorOne.setControl(new VoltageOut(voltage * 12).withEnableFOC(true));
        motorTwo.setControl(new VoltageOut(voltage * 12).withEnableFOC(true));
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

    public void setNewCurrentLimit(double stator, double supply) {
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = stator;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLimit = supply;
        limitConfig.SupplyCurrentLimitEnable = true;
        motorOne.getConfigurator().apply(limitConfig);
        motorTwo.getConfigurator().apply(limitConfig);
    }

    public double getSubsystemStatorCurrent() {
        return inputs.spindexerOneStatorCurrent + inputs.spindexerTwoStatorCurrent;
    }

    public double getSubsystemSupplyCurrent() {
        return inputs.spindexerOneSupplyCurrent + inputs.spindexerTwoSupplyCurrent;
    }

    @Override
    public void updateInputs() {
        inputs.spindexerOneVelocity = motorOne.getVelocity().getValueAsDouble();
        inputs.spindexerOneStatorCurrent = motorOne.getStatorCurrent().getValueAsDouble();
        inputs.spindexerOneSupplyCurrent = motorOne.getSupplyCurrent().getValueAsDouble();
        inputs.spindexerTwoVelocity = motorTwo.getVelocity().getValueAsDouble();
        inputs.spindexerTwoStatorCurrent = motorTwo.getStatorCurrent().getValueAsDouble();
        inputs.spindexerTwoSupplyCurrent = motorTwo.getSupplyCurrent().getValueAsDouble();
    }
}
