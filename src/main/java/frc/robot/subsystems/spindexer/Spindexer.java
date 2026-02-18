package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class Spindexer extends SubsystemBase implements SpindexerIO {
    TalonFX motor = new TalonFX(IdConstants.SPINDEXER_ID);

    private double power = 0.0;
    private int ballCount = 0;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
    private boolean wasAboveThreshold = false;

    public Spindexer() {
        // SmartDashboard.putData("Turn on Spindexer", new InstantCommand(()->
        // turnOnSpindexer()));
    }

    @Override
    public void periodic() {
        power = SmartDashboard.getNumber("Spindexer Power", power);
        SmartDashboard.putNumber("Spindexer Power", power);

        motor.set(power);
        updateInputs();

        boolean isAboveThreshold = inputs.spindexerVelocity >= SpindexerConstants.spindexerVelocityWithBall;
        if (wasAboveThreshold && !isAboveThreshold && power > 0.1) {
            ballCount++;
        }
        wasAboveThreshold = isAboveThreshold;
    }

    /**
     * @return
     */
    public void maxSpindexer() {
        power = 0.5;
    }

    public void stopSpindexer() {
        power = 0.0;
    }

    @Override
    public void updateInputs() {
        inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble();
        inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
    }

}
