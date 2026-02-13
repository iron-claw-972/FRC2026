package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.Follower; // Added for following
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase implements ShooterIO {
    
    private TalonFX motorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX motorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    public VelocityVoltage flywheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    
    // Goal Velocity
    private double targetSpeedMPS = 0;

    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double torqueCurrentControlTolerance = 5.0;
    private double torqueCurrentDebounceTime = 0.025;

    private Debouncer torqueCurrentDebouncer = new Debouncer(torqueCurrentDebounceTime, DebounceType.kFalling);
    @AutoLogOutput private long launchCount = 0;
    private boolean lastTorqueCurrentControl = false; 

    public Shooter(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.15; 
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.03;
        config.Slot0.kV = 0.20; 

        config.MotionMagic = new MotionMagicConfigs().withMotionMagicCruiseVelocity(90).withMotionMagicAcceleration(130);

        motorLeft.getConfigurator().apply(config);
        motorRight.getConfigurator().apply(config);

        motorRight.getConfigurator().apply(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        );

        motorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        );

        motorRight.setControl(new Follower(motorLeft.getDeviceID(), MotorAlignmentValue.Opposed));

        SmartDashboard.putData("Turn on shooter", new InstantCommand(() -> setShooter(ShooterConstants.SHOOTER_VELOCITY)));
        SmartDashboard.putData("Turn off Shooter", new InstantCommand(() -> setShooter(0)));
    }

    public void periodic(){
        updateInputs();
        Logger.processInputs("Shooter", inputs);
        countLaunch(Units.rotationsToRadians(targetSpeedMPS));

        double rps = Units.radiansToRotations(targetSpeedMPS / (ShooterConstants.SHOOTER_LAUNCH_DIAMETER / 2.0));
        motorLeft.setControl(flywheelVelocityVoltage.withVelocity(rps));
    }

    private void countLaunch(double velocityRadsPerSec) {
        boolean inTolerance =
            Math.abs(Units.rotationsToRadians(getShooterVelcoity()) - velocityRadsPerSec)
                <= torqueCurrentControlTolerance;
        
        boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance); 

        if (!torqueCurrentControl && lastTorqueCurrentControl) {
            launchCount++;
        }
        lastTorqueCurrentControl = torqueCurrentControl;

        Logger.recordOutput("Shooter/Setpoint", velocityRadsPerSec);
    }

    public void deactivateShooterAndFeeder() {
        setShooter(0);
    }

    public void setShooter(double velocityMPS) {
        targetSpeedMPS = velocityMPS;
    }

    public double getShooterVelcoity() {
        return inputs.leftShooterVelocity; 
    }

    public boolean atTargetVelocity(){
        return Math.abs(getShooterVelcoity() - targetSpeedMPS) < 0.1; // Tolerance of 0.1 mps
    }

    public void updateInputs() {
        inputs.leftShooterVelocity = Units.rotationsToRadians(motorLeft.getVelocity().getValueAsDouble()) * ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
        inputs.rightShooterVelocity =Units.rotationsToRadians(motorRight.getVelocity().getValueAsDouble()) * ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
        inputs.leftShooterCurrent = motorLeft.getStatorCurrent().getValueAsDouble();
        inputs.rightShooterCurrent = motorRight.getStatorCurrent().getValueAsDouble();
    }
}