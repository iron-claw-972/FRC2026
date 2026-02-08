package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
    
    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

    //rotations/sec

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;

    public double shooterPower = 0.5;
    //Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // for tracking current phase: used to adjust the setting
    private FlywheelPhase phase;

    private double torqueCurrentControlTolerance = 20.0;
    private double torqueCurrentDebounceTime = 0.025;
    private double atGoalStateDebounceTime = 0.2;
    private Debouncer torqueCurrentDebouncer = new Debouncer(torqueCurrentDebounceTime, DebounceType.kFalling);
    private Debouncer atGoalStateDebouncer = new Debouncer(atGoalStateDebounceTime, DebounceType.kFalling);
    private boolean atGoal = false;
    @AutoLogOutput private long launchCount = 0;
    private boolean lastTorqueCurrentControl = false;

    public enum FlywheelPhase {
        BANG_BANG,
        CONSTANT_TORQUE,
        START_UP
    }    

    public Shooter(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 676767.0; //tune p value
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0.12; //Maximum rps = 100 --> 12V/100rps
        
        config.TorqueCurrent.PeakForwardTorqueCurrent = 40; // this is the constant torque velocity
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0; // we are making this a BANG BANG controller for talon fx
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;

        shooterMotorLeft.getConfigurator().apply(config);
        shooterMotorRight.getConfigurator().apply(config);
        
        shooterMotorRight.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        shooterMotorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        );

        // set start up for phase initially:
        phase = FlywheelPhase.START_UP;

        SmartDashboard.putData("Turn on shooter", new InstantCommand(() -> setShooter(ShooterConstants.SHOOTER_VELOCITY)));
        SmartDashboard.putData("Turn ALL off", new InstantCommand(() -> deactivateShooterAndFeeder()));
        SmartDashboard.putData("Turn off Shooter", new InstantCommand(() -> setShooter(0)));
    }

    public void periodic(){
        // updateInputs();
        // runVelocity(Units.rotationsToRadians(getShooterVelcoity()));
        // SmartDashboard.putNumber("Shot Power", shooterPower);
        // shooterPower = SmartDashboard.getNumber("Shot Power", shooterPower);

        // if (phase == FlywheelPhase.BANG_BANG) { // shooter target speed is in RPS
        //     // Duty-cycle bang-bang (apply to both)
        //     shooterMotorLeft.setControl(new VelocityDutyCycle(shooterTargetSpeed)); // spin as fast as possible shooter target speed
        //     shooterMotorRight.setControl(new VelocityDutyCycle(shooterTargetSpeed)); // same here
        // } else {
        //     // Torque-current bang-bang
        //     shooterMotorLeft.setControl(new VelocityTorqueCurrentFOC(shooterTargetSpeed)); // apply constant torque current
        //     shooterMotorRight.setControl(new VelocityTorqueCurrentFOC(shooterTargetSpeed)); // again
        // }

        shooterMotorLeft.set(-shooterPower);
        shooterMotorRight.set(-shooterPower);
    }

    /** Run closed loop at the specified velocity. */
    private void runVelocity(double velocityRadsPerSec) {
        boolean inTolerance =
            Math.abs(Units.rotationsToRadians(getShooterVelcoity()) - velocityRadsPerSec)
                <= torqueCurrentControlTolerance;
        boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
        atGoal = atGoalStateDebouncer.calculate(inTolerance);

        if (!torqueCurrentControl && lastTorqueCurrentControl) {
        launchCount++;
        }
        lastTorqueCurrentControl = torqueCurrentControl;

        phase =
            torqueCurrentControl
                ? FlywheelPhase.BANG_BANG
                : FlywheelPhase.CONSTANT_TORQUE;
        shooterTargetSpeed = Units.radiansToRotations(velocityRadsPerSec);
        Logger.recordOutput("Shooter/Setpoint", velocityRadsPerSec);
    }


    public void deactivateShooterAndFeeder() {
        setShooter(0);
        System.out.println("Shooter deactivated");
    }

    public void setShooter(double velocityRPS) {
        shooterTargetSpeed = velocityRPS;
        System.out.println("Shooter is working");
    }

    public void setShooterPower(double power){
        this.shooterPower = power;
    }

    public double getShooterVelcoity() {
        return inputs.leftShooterVelocity; // assuming they are the same rn
    }

    public void updateInputs() {
        inputs.leftShooterVelocity = shooterMotorLeft.getVelocity().getValueAsDouble();
        inputs.rightShooterVelocity = shooterMotorRight.getVelocity().getValueAsDouble();
        inputs.leftShooterCurrent = shooterMotorLeft.getStatorCurrent().getValueAsDouble();
        inputs.rightShooterCurrent = shooterMotorRight.getStatorCurrent().getValueAsDouble();
    }
}