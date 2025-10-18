package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.IdConstants;

public class shooterReal extends shooterBase {
    
    private TalonFX shooterMotor = new TalonFX(IdConstants.SHOOTER_ID);
    private TalonFX feederMotor = new TalonFX(IdConstants.FEEDER_ID);

    //rotations/sec
    private double shooterTargetSpeed = 0;
    private double feederPower = 0;

    //Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    public shooterReal(){
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1; //tune p value
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0.12; //Maximum rps = 100 --> 12V/100rps
        shooterMotor.getConfigurator().apply(config);
        
        shooterMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        feederMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );
    }

    @Override
    public void periodic(){
        shooterMotor.setControl(voltageRequest.withVelocity(shooterTargetSpeed));
        feederMotor.set(feederPower);
    }

    @Override
    public void setFeeder(double power){
        feederPower = power;
    }

    @Override
    public void setShooter(double power){
        //Maximum velocity = 100
        shooterTargetSpeed = power * 100.0;
    }

    @Override
    public double getShooterVelcoity(){
        return Units.radiansToDegrees(shooterMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public double getFeederVelocity(){
        return Units.radiansToDegrees(feederMotor.getVelocity().getValueAsDouble());
    }
}
