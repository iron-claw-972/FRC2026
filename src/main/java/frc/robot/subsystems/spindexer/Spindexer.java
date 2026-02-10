package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;
import frc.robot.subsystems.spindexer.SpindexerIO;

public class Spindexer extends SubsystemBase implements SpindexerIO{
    TalonFX motor = new TalonFX(IdConstants.SPINDEXER_ID);

    private double power = 0.0;
    private double runPower = 0.0;

    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer(){
        //SmartDashboard.putData("Turn on Spindexer", new InstantCommand(()-> turnOnSpindexer()));
    }

    @Override
    public void periodic() {
        runPower = SmartDashboard.getNumber("Spindexer Power", runPower);
        SmartDashboard.putNumber("Spindexer Power", runPower);

        motor.set(power);
    }

    public void turnOnSpindexer(){
        power = runPower;
    }

    public void stopSpindexer(){
        power = 0.0;
    }

    @Override
    public void updateInputs() {
        inputs.spindexerVelocity = motor.getVelocity().getValueAsDouble();
        inputs.spindexerCurrent = motor.getStatorCurrent().getValueAsDouble();
    }

}
