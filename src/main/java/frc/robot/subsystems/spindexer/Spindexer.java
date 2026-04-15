package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Spindexer extends SubsystemBase {

    private double power = 0.0;
    public int ballCount = 0;
    private boolean wasSpindexerSlow = false;
    private SpindexerState state = SpindexerState.STOPPED;
    private boolean reversing = false;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public boolean noIndexing = false;

    private SpindexerIO io;


    public Spindexer(SpindexerIO io) {
        this.io = io;
        io.updateInputs(inputs);

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
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);

        if (resetPos == null) {
            io.setPositionRaw(0.1 * gearRatio);
            resetPos = (inputs.spindexerPosition / gearRatio) % 1.0;
            resetPID.reset();
        }

        if (state == SpindexerState.MAX) {
            io.setControl(new DutyCycleOut(SpindexerConstants.spindexerMaxPower).withEnableFOC(true));
            // io.setControl(new ToqueCurrentFOC(Output)); TODO this is better, use torquecurerntFOC, we want to control torque not speed 
            reversing = false;
        } else if (state == SpindexerState.REVERSE) {
            io.setControl(new VoltageOut(SpindexerConstants.spindexerReversePower * 12).withEnableFOC(true)); //TODO voltageout, direct upgrade from dutycycle, which multiplies by battery voltage, so worse when driving around
            reversing = true;
        } else if (state == SpindexerState.STOPPED) {
            io.setControl(new DutyCycleOut(0.0).withEnableFOC(true));
            reversing = false;
        } else if (state == SpindexerState.RESET && resetPos != null) {
            io.setControl(new DutyCycleOut(resetPID.calculate((inputs.spindexerPosition / gearRatio) % 1.0, resetPos)).withEnableFOC(true));
        } else {
            io.setControl(new DutyCycleOut(power).withEnableFOC(true));
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
        }
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
        io.setNewCurrentLimit(newCurrentLimit);
    }


    private Double resetPos;
    private PIDController resetPID = new PIDController(4.0, 0.0, 0);

    private final double gearRatio = 27.0 / 1.0; //spindexer spins once for every 27 motor spins

    

}
