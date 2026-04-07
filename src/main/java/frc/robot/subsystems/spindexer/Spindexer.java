package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Spindexer extends SubsystemBase implements SpindexerIO {
    private TalonFX motor = new TalonFX(IdConstants.SPINDEXER_ID, Constants.CANIVORE_SUB);

    private double power = 0.0;
    public int ballCount = 0;
    private boolean wasSpindexerSlow = false;
    private SpindexerState state = SpindexerState.STOPPED;
    private boolean reversing = false;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private Debouncer noBallsDebouncer = new Debouncer(SpindexerConstants.NO_BALLS_DEBOUNCE_TIME, DebounceType.kRising);
    public boolean noIndexing = false;

    public Spindexer() {
        updateInputs();

        // configure current limit
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = SpindexerConstants.CURRENT_SPIKE_LIMIT;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLowerLimit = SpindexerConstants.currentLimit;
        limitConfig.SupplyCurrentLowerTime = 1.5;
        motor.getConfigurator().apply(limitConfig);

        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putData("Spindexer Run Forward", new InstantCommand(() -> maxSpindexer()));
            SmartDashboard.putData("Spindexer Run Reverse", new InstantCommand(() -> reverseSpindexer()));
            SmartDashboard.putData("Spindexer Stop", new InstantCommand(() -> stopSpindexer()));
        }

        resetPID.setTolerance(0.05);
    }

    public enum SpindexerState {
        MAX,
        REVERSE,
        STOPPED,
        RESET,
        CUSTOM,
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Spindexer", inputs);

        if (resetPos == null) {
            motor.setPosition(0.1 * gearRatio);
            resetPos = (motor.getPosition().getValueAsDouble() / gearRatio) % 1.0;
            resetPID.reset();
        }

        if (state == SpindexerState.MAX) {
            motor.setControl(new DutyCycleOut(SpindexerConstants.spindexerMaxPower).withEnableFOC(true));
            reversing = false;
        } else if (state == SpindexerState.REVERSE) {
            motor.setControl(new DutyCycleOut(SpindexerConstants.spindexerReversePower).withEnableFOC(true));
            reversing = true;
        } else if (state == SpindexerState.STOPPED) {
            motor.setControl(new DutyCycleOut(0.0).withEnableFOC(true));
            reversing = false;
        } else if (state == SpindexerState.RESET && resetPos != null) {
            motor.setControl(new DutyCycleOut(resetPID.calculate((motor.getPosition().getValueAsDouble() / gearRatio) % 1.0, resetPos)).withEnableFOC(true));
        } else {
            motor.setControl(new DutyCycleOut(power).withEnableFOC(true));
            reversing = false;
        }


        // scale threshold based on power
        double velocityThreshold = SpindexerConstants.spindexerVelocityWithBall * power;
        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putNumber("Spindexer Ball Count", ballCount);

            SmartDashboard.putBoolean("Spindexer Running", state == SpindexerState.MAX || state == SpindexerState.CUSTOM);
            SmartDashboard.putBoolean("Spindexer Has Ball", ballCount > 0);
        }

        boolean isSpindexerSlow = inputs.spindexerVelocity < velocityThreshold;
        if (wasSpindexerSlow && !isSpindexerSlow && power > 0.1) {
            ballCount++;
        }
        wasSpindexerSlow = isSpindexerSlow;

        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putBoolean("Spindexer Reversing", state == SpindexerState.REVERSE);
            SmartDashboard.putBoolean("Spindexer Has Balls", inputs.spindexerCurrent > SpindexerConstants.NO_BALLS_THRESHOLD_CURRENT);
        }
    }

    public boolean hasNoBalls() {
        // TODO: tune the threshold of course
        double current = motor.getStatorCurrent().getValueAsDouble();
        boolean hasBalls = current > SpindexerConstants.NO_BALLS_THRESHOLD_CURRENT;
        boolean noBalls = noBallsDebouncer.calculate(!hasBalls);
        return noBalls;
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

    public void resetSpindexer() {
        state = SpindexerState.RESET;
    }

    public void resetResetAngle() {
        resetPos = null;
    }

    public double getStatorCurrent() {
        return inputs.spindexerCurrent;
    }

    public void setNewCurrentLimit(double newCurrentLimit) {
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = SpindexerConstants.CURRENT_SPIKE_LIMIT;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLowerLimit = newCurrentLimit;
        limitConfig.SupplyCurrentLowerTime = 1.5;
        motor.getConfigurator().apply(limitConfig);
    }

    @Override
    public void updateInputs() {
        inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble(); //SpindexerConstants.gearRatio;
        inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
    }

    private Double resetPos;
    private PIDController resetPID = new PIDController(4.0, 0.0, 0);

    private final double gearRatio = 27.0 / 1.0; //spindexer spins once for every 27 motor spins

    

}
