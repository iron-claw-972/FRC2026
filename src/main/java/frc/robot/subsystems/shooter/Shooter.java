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
<<<<<<< HEAD
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
=======
>>>>>>> parent of ae21e1f7 (Merge branch 'arnavs-mega-velocityvoltage-turret' into beta-bot)
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        config.Slot0.kP = 0.15; //tune p value
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.125; //Maximum rps = 100 --> 12V/100rps
        shooterMotorLeft.getConfigurator().apply(config);
        shooterMotorRight.getConfigurator().apply(config);
        
        shooterMotorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        shooterMotorRight.getConfigurator().apply(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        );

        SmartDashboard.putData("Turn on shooter", new InstantCommand(()-> setShooter(12.0)));
    }

    @Override
    public void periodic(){
        updateInputs();

        powerModifier = SmartDashboard.getNumber("shooter power modifier", powerModifier);
        SmartDashboard.putNumber("shooter power modifier", powerModifier);
        shooterMotorLeft.setControl(voltageRequest.withVelocity(Units.radiansToRotations(shooterTargetSpeed / (ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2))));
        shooterMotorRight.setControl(voltageRequest.withVelocity(Units.radiansToRotations(shooterTargetSpeed / (ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2))));   
        
        Logger.recordOutput("Shooter/targetVelocity", shooterTargetSpeed);
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
        inputs.shooterSpeedLeft = Units.rotationsToRadians(shooterMotorLeft.getVelocity().getValueAsDouble()) * ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
        inputs.shooterSpeedRight = Units.rotationsToRadians(shooterMotorRight.getVelocity().getValueAsDouble())* ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
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