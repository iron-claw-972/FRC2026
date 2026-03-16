package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.util.HubActive;

public class Shooter extends SubsystemBase implements ShooterIO {
    
    private TalonFX shooterMotorLeft = new TalonFX(IdConstants.SHOOTER_LEFT_ID, Constants.CANIVORE_SUB);
    private TalonFX shooterMotorRight = new TalonFX(IdConstants.SHOOTER_RIGHT_ID, Constants.CANIVORE_SUB);

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;

    // Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    double powerModifier = 1.05; // TESTED

    public Shooter(){
        updateInputs();
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 1.0; //tune p value
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

        SmartDashboard.putData("OPERATOR: Increase Shooter Modifier by 0.05", new InstantCommand(()-> increaseShooterModifier(0.05)));
        SmartDashboard.putData("OPERATOR: Decrease Shooter Modifier by 0.05", new InstantCommand(()-> decreaseShooterModifier(0.05)));
    }

    @Override
    public void periodic(){
        updateInputs();

        // shooterTargetSpeed = SmartDashboard.getNumber("Shooter Setpoint", shooterTargetSpeed);
        // SmartDashboard.putNumber("Shooter Setpoint", shooterTargetSpeed);
        // Convert to RPS
        double targetVelocityRPS = Units.radiansToRotations(shooterTargetSpeed / (ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2)) * powerModifier;

        SmartDashboard.putNumber("OPERATOR: Shooter Modifier", powerModifier);

        SmartDashboard.putNumber("Target Velocity RPS", targetVelocityRPS);
        SmartDashboard.putNumber("Shooter Motor RPS", shooterMotorLeft.getVelocity().getValueAsDouble());

        // Sets the motor control to target velocity
        shooterMotorLeft.setControl(voltageRequest.withVelocity(targetVelocityRPS));
        shooterMotorRight.setControl(voltageRequest.withVelocity(targetVelocityRPS));   
        
        Logger.recordOutput("Shooter/realVelocity", shooterMotorLeft.getVelocity().getValueAsDouble() * ShooterConstants.SHOOTER_LAUNCH_DIAMETER);
        Logger.recordOutput("Shooter/targetVelocity", shooterTargetSpeed);

        SmartDashboard.putString("WON AUTO?", (HubActive.wonAuto()) ? "WOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOON" : "lost");
    }

    /**
     * Sets the target speed of the shooter
     * @param linearVelocityMps
     */
    public void setShooter(double linearVelocityMps) {
        shooterTargetSpeed = linearVelocityMps;
    }

    /**@return velocity in m/s */
    public double getShooterVelocity(){
        return inputs.shooterSpeedLeft;
    }

    public void increaseShooterModifier(double mod) {
        powerModifier += mod;
    }

    public void decreaseShooterModifier(double mod) {
        powerModifier -= mod;
    }

    @Override
    public void updateInputs(){
        inputs.shooterSpeedLeft = Units.rotationsToRadians(shooterMotorLeft.getVelocity().getValueAsDouble()) * ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
        inputs.shooterSpeedRight = Units.rotationsToRadians(shooterMotorRight.getVelocity().getValueAsDouble())* ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
        inputs.shooterCurrentLeft = shooterMotorLeft.getStatorCurrent().getValueAsDouble();
        inputs.shooterCurrentRight = shooterMotorRight.getStatorCurrent().getValueAsDouble();


        Logger.processInputs("Shooter", inputs);
    }

    /**
     * @return Whether the shooter is at the target speed with tolerance of 1 m/s
     */
    public boolean atTargetSpeed(){
        return Math.abs(getShooterVelocity() - shooterTargetSpeed) < 1.0;
    }

    /**
     * @return Gets the target velocity in m/s
     */
    @AutoLogOutput(key="Shooter/TargetSpeed")
    public double getTargetVelocity(){
        return shooterTargetSpeed;
    }
}
