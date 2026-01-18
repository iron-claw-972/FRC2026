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

public class ShooterReal extends ShooterBase implements ShooterIO {
    
    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    private TalonFX feederMotor = new TalonFX(IdConstants.FEEDER_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private LaserCan sensor = new LaserCan(IdConstants.SHOOTER_SENSOR_ID);

    //rotations/sec

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;
    private double feederPower = 0;


    public boolean shooterAtMaxSpeed = false;
    public boolean ballDetected = false;
    //Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public ShooterReal(){

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
            new InstantCommand(()-> setShooter(ShooterConstants.SHOOTER_VELOCITY)),
            new WaitCommand(0.5),
            new InstantCommand(()-> setFeeder(ShooterConstants.FEEDER_RUN_POWER))
        ));
        SmartDashboard.putData("Stop Shooting", new InstantCommand(()-> deactivateShooterAndFeeder()));
        SmartDashboard.putData("Turn own main shooter motor", new InstantCommand(()-> setShooter(ShooterConstants.SHOOTER_VELOCITY)));
        SmartDashboard.putData("Turn own feeder motor", new InstantCommand(()-> setFeeder(ShooterConstants.FEEDER_RUN_POWER)));
    }

    @Override
    public void periodic(){
        updateInputs();
        shooterMotorLeft.setControl(voltageRequest.withVelocity(shooterTargetSpeed));
        shooterMotorRight.setControl(voltageRequest.withVelocity(shooterTargetSpeed));
        feederMotor.set(feederPower);

        ballDetected();
        //SmartDashboard.putNumber("Sensor Distance", sensor.getMeasurement().distance_mm);
        
        shooterAtMaxSpeed = atMaxSpeed();
    }

    public void deactivateShooterAndFeeder() {
        setFeeder(0);
        setShooter(0);
        System.out.println("Shooter deactivated");
    }

    public boolean atMaxSpeed() {
        double leftRps = shooterMotorLeft.getVelocity().getValueAsDouble();
        double rightRps = shooterMotorRight.getVelocity().getValueAsDouble();
        double average = (leftRps + rightRps) / 2.0;
        return average >= shooterTargetSpeed * 0.95;
    }

    public boolean atTargetSpeed() {
        return ((shooterTargetSpeed - shooterMotorLeft.getVelocity().getValueAsDouble())) < ShooterConstants.EXIT_VELOCITY_TOLERANCE;
    }

    @Override
    public void setFeeder(double power){
        System.out.println("VELOCITY: " + getShooterVelcoity()); 
        feederPower = power;
    }

    public void setShooter(double linearVelocityMps) {
        double wheelCircumference = Math.PI * ShooterConstants.SHOOTER_LAUNCH_DIAMETER;
        shooterTargetSpeed = linearVelocityMps * ShooterConstants.SHOOTER_GEAR_RATIO / wheelCircumference; // rps
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
    public boolean ballDetected() {
        Measurement measurement = sensor.getMeasurement();
        ballDetected =  measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 45;
        return ballDetected;
    }

    @Override
    public void updateInputs(){
        inputs.shooterSpeedLeft = shooterMotorLeft.getVelocity().getValueAsDouble();
        inputs.shooterSpeedRight = shooterMotorRight.getVelocity().getValueAsDouble();
        inputs.feederSpeed = feederMotor.getVelocity().getValueAsDouble();
        inputs.sensorDistance = sensor.getMeasurement().distance_mm;

        Logger.processInputs("Shooter", inputs);
    }
}
