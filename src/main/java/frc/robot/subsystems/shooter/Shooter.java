package frc.robot.subsystems.shooter;

import java.lang.annotation.Target;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase implements ShooterIO {
    
    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.RIO_CAN);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.RIO_CAN);

    private TalonFX feederMotor = new TalonFX(IdConstants.FEEDER_ID, Constants.RIO_CAN);

    //rotations/sec

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;
    private double feederPower = 0;
    //Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.2; //tune p value
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0.12; //Maximum rps = 100 --> 12V/100rps
        shooterMotorLeft.getConfigurator().apply(config);
        shooterMotorRight.getConfigurator().apply(config);
        
        shooterMotorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        shooterMotorRight.getConfigurator().apply(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        );

        feederMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        SmartDashboard.putData("Turn on shooter", new InstantCommand(() -> setShooter(ShooterConstants.SHOOTER_VELOCITY)));
        SmartDashboard.putData("Turn on feeder", new InstantCommand(() -> setFeeder(ShooterConstants.FEEDER_RUN_POWER)));
        SmartDashboard.putData("Turn ALL off", new InstantCommand(() -> deactivateShooterAndFeeder()));
        SmartDashboard.putData("Turn off Shooter", new InstantCommand(() -> setShooter(0)));
        SmartDashboard.putData("Turn off feeder", new InstantCommand(() -> setFeeder(0)));
    }

    public void periodic(){
        updateInputs();

        shooterMotorLeft.setControl(voltageRequest.withVelocity(shooterTargetSpeed));
        shooterMotorRight.setControl(voltageRequest.withVelocity(shooterTargetSpeed));
        feederMotor.set(feederPower);
    }

    public void deactivateShooterAndFeeder() {
        setFeeder(0);
        setShooter(0);
        System.out.println("Shooter deactivated");
    }
    public void setFeeder(double power){
        System.out.println("VELOCITY: " + getShooterVelcoity()); 
        feederPower = power;
    }

    public void setShooter(double linearVelocityMps) {
        double wheelCircumference = Math.PI * ShooterConstants.SHOOTER_LAUNCH_DIAMETER;
        System.out.println("PRINTING WHEEEEEEEEEEEEL CIRUM:" + wheelCircumference);
        shooterTargetSpeed = linearVelocityMps * ShooterConstants.SHOOTER_GEAR_RATIO / wheelCircumference; // rps
        System.out.println("Shooter is working");
    }

    public double getFeederVelocity() {
        return inputs.feederVelocity;
    }

    public double getShooterVelcoity() {
        return inputs.leftShooterVelocity; // assuming they are the same rn
    }

    public void updateInputs() {
        inputs.leftShooterVelocity = shooterMotorLeft.getVelocity().getValueAsDouble();
        inputs.rightShooterVelocity = shooterMotorRight.getVelocity().getValueAsDouble();
        inputs.feederVelocity = feederMotor.getVelocity().getValueAsDouble();
        inputs.leftShooterCurrent = shooterMotorLeft.getStatorCurrent().getValueAsDouble();
        inputs.rightShooterVelocity = shooterMotorRight.getStatorCurrent().getValueAsDouble();
        inputs.feederVelocity = feederMotor.getStatorCurrent().getValueAsDouble();
    }
}
