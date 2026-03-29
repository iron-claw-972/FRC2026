package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
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

    public Spindexer() {
        updateInputs();

        // configure current limit
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = SpindexerConstants.CURRENT_SPIKE_LIMIT;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLowerLimit = SpindexerConstants.currentLimit;
        limitConfig.SupplyCurrentLowerTime = 1.5;
        motor.getConfigurator().apply(limitConfig);

        SmartDashboard.putData("Spindexer Run Forward", new InstantCommand(() -> maxSpindexer()));
        SmartDashboard.putData("Spindexer Run Reverse", new InstantCommand(() -> reverseSpindexer()));
        SmartDashboard.putData("Spindexer Stop", new InstantCommand(() -> stopSpindexer()));
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
            resetPos = (motor.getPosition().getValueAsDouble() % gearRatio);
            resetPID.reset();
        }

        if (state == SpindexerState.MAX) {
            motor.set(SpindexerConstants.spindexerMaxPower);
            reversing = false;
        } else if (state == SpindexerState.REVERSE) {
            motor.set(SpindexerConstants.spindexerReversePower);
            reversing = true;
        } else if (state == SpindexerState.STOPPED) {
            motor.set(0.0);
            reversing = false;
        } else if (state == SpindexerState.RESET && resetPos != null) {
            motor.set(resetPID.calculate((motor.getPosition().getValueAsDouble() % gearRatio), resetPos));
        } else {
            motor.set(power);
            reversing = false;
        }

        // scale threshold based on power
        double velocityThreshold = SpindexerConstants.spindexerVelocityWithBall * power;
        SmartDashboard.putNumber("Spindexer Ball Count", ballCount);

        SmartDashboard.putBoolean("Spindexer Running", state == SpindexerState.MAX || state == SpindexerState.CUSTOM);
        SmartDashboard.putBoolean("Spindexer Has Ball", ballCount > 0);

        boolean isSpindexerSlow = inputs.spindexerVelocity < velocityThreshold;
        if (wasSpindexerSlow && !isSpindexerSlow && power > 0.1) {
            ballCount++;
        }
        wasSpindexerSlow = isSpindexerSlow;

        SmartDashboard.putBoolean("Spindexer Jamming", reversing);

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

    @Override
    public void updateInputs() {
        inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble(); //SpindexerConstants.gearRatio;
        inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
        Logger.processInputs("Spindexer", inputs);
    }

    private Double resetPos;
    private PIDController resetPID = new PIDController(0.5, 0.1, 0);

    private final double gearRatio = 27.0 / 1.0; //spindexer spins once for every 27 motor spins

    

}
