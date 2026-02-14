package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Shooter extends SubsystemBase implements ShooterIO {
    
    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;
    private double feederPower = 0;


    public boolean shooterAtMaxSpeed = false;
    public boolean ballDetected = false;
    //Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    double powerModifier;

    public Shooter(){

        updateInputs();
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.1; //tune p value
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
    }

    @Override
    public void periodic(){
        updateInputs();

        powerModifier = SmartDashboard.getNumber("shooter power modifier", powerModifier);
        SmartDashboard.putNumber("shooter power modifier", powerModifier);
        shooterMotorLeft.setControl(voltageRequest.withVelocity(shooterTargetSpeed * powerModifier));
        shooterMotorRight.setControl(voltageRequest.withVelocity(shooterTargetSpeed * powerModifier));          
    }

    public void deactivateShooterAndFeeder() {
        setFeeder(0);
        setShooter(0);
    }

    public void setFeeder(double power){
        // System.out.println("VELOCITY: " + getShooterVelcoity()); 
        feederPower = power;
    }

    public void setShooter(double linearVelocityMps) {
        shooterTargetSpeed = linearVelocityMps;
    }

    /**@return velocity in m/s */
    public double getShooterVelcoity(){
        return inputs.shooterSpeedLeft;
    }

    @Override
    public void updateInputs(){
        inputs.shooterSpeedLeft = Units.rotationsToRadians(shooterMotorLeft.getVelocity().getValueAsDouble());
        inputs.shooterSpeedRight = Units.rotationsToRadians(shooterMotorRight.getVelocity().getValueAsDouble());
        Logger.processInputs("Shooter", inputs);
    }

    public boolean atTargetSpeed(){
        return Math.abs(getShooterVelcoity() - shooterTargetSpeed) < 1.0;
    }

    @AutoLogOutput(key="Shooter/TargetSpeed")
    public double getTargetVelocity(){
        return Units.rotationsToRadians(shooterTargetSpeed);
    }
}