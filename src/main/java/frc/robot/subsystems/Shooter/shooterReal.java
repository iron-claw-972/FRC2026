package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

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
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public class shooterReal extends shooterBase implements ShooterIO {
    
    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_ONE_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_TWO_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private TalonFX feederMotor = new TalonFX(IdConstants.FEEDER_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private LaserCan sensor = new LaserCan(IdConstants.SHOOTER_SENSOR_ID);

    //rotations/sec

    // Goal Velocity / Double the Circumfrence
    private double shooterTargetSpeed = ShooterConstants.SHOOTER_VELOCITY / 2 * Math.PI * ShooterConstants.SHOOTER_LAUNCH_DIAMETER;
    private double feederPower = 0;


    public boolean shooterAtMaxSpeed = false;

    //Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public shooterReal(){

        updateInputs();
        //Sensor configs
        try {
            sensor.setRangingMode(RangingMode.SHORT);
            sensor.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
            sensor.setRegionOfInterest(new RegionOfInterest(-4, -4, 8, 8));
        } catch (ConfigurationFailedException e) {
            DriverStation.reportError("Indexer LaserCan configuration error", true);
        }
        
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

        SmartDashboard.putData("Shoot Game Piece", new SequentialCommandGroup(
            new InstantCommand(()-> setShooter(ShooterConstants.SHOOTER_RUN_POWER)),
            new WaitCommand(0.5),
            new InstantCommand(()-> setFeeder(ShooterConstants.FEEDER_RUN_POWER))
        ));
        SmartDashboard.putData("Stop Shooting", new InstantCommand(()-> deactivateShooterAndFeeder()));
        SmartDashboard.putData("Turn own main shooter motor", new InstantCommand(()-> setShooter(ShooterConstants.SHOOTER_RUN_POWER)));
        SmartDashboard.putData("Turn own feeder motor", new InstantCommand(()-> setFeeder(ShooterConstants.FEEDER_RUN_POWER)));
    }

    @Override
    public void periodic(){
        updateInputs();
        shooterMotorLeft.setControl(voltageRequest.withVelocity(shooterTargetSpeed));
        shooterMotorRight.setControl(voltageRequest.withVelocity(shooterTargetSpeed));
        feederMotor.set(feederPower);

        if (shooterMotorLeft.getVelocity().getValueAsDouble() >= shooterTargetSpeed * 0.95) {
            shooterAtMaxSpeed = true;
        } else {
            shooterAtMaxSpeed = false;
        }
    }

    public void deactivateShooterAndFeeder() {
        setFeeder(0);
        setShooter(0);
        System.out.println("Shooter deactivated");
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
        return inputs.shooterSpeedLeft;
    }

    @Override
    public double getFeederVelocity(){
        return inputs.feederSpeed;
    }

    @AutoLogOutput
    public boolean ballDetected(){
        Measurement measurement = sensor.getMeasurement();
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && measurement.distance_mm <= 1000 * 0.3;
    }

    @Override
    public void updateInputs(){
        inputs.shooterSpeedLeft = shooterMotorLeft.getVelocity().getValueAsDouble();
        inputs.shooterSpeedRight = shooterMotorRight.getVelocity().getValueAsDouble();
        inputs.feederSpeed = feederMotor.getVelocity().getValueAsDouble();
        inputs.sensorDistance = sensor.getMeasurement().distance_mm;
    }
}
