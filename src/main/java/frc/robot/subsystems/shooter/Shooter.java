package frc.robot.subsystems.shooter;

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
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

public class Shooter  {
    
    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private TalonFX feederMotor = new TalonFX(IdConstants.FEEDER_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    //rotations/sec

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;
    private double feederPower = 0;
    //Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    public Shooter(){
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
        shooterTargetSpeed = linearVelocityMps * ShooterConstants.SHOOTER_GEAR_RATIO / wheelCircumference; // rps
    }

    public double getFeederVelocity() {
        return feederMotor.getVelocity().getValueAsDouble();
    }
    public double getShooterVelcoity() {
        return shooterMotorLeft.getVelocity().getValueAsDouble();
    }
}
