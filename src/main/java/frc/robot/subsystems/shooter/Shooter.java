package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

    //TODO Add current limits

    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;

    // Velocity in rotations per second
    VelocityVoltage voltageRequest = new VelocityVoltage(0);

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    double powerModifier = 1.00;

    public Shooter() {
        updateInputs();
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0.5; // 0.5 stable
        config.Slot0.kI = 0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.125; //Maximum rps = 100 --> 12V/100rps

        config.CurrentLimits
        .withSupplyCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);

        shooterMotorLeft.getConfigurator().apply(config);
        shooterMotorRight.getConfigurator().apply(config);
        
        shooterMotorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );

        shooterMotorRight.getConfigurator().apply(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
        );

        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        limitConfig.StatorCurrentLimitEnable = true;
        shooterMotorLeft.getConfigurator().apply(limitConfig);
        shooterMotorRight.getConfigurator().apply(limitConfig);

        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putData("Turn on shooter", new InstantCommand(()-> setShooter(12.0)));
        }
    }

    @Override
    public void periodic(){
        updateInputs();

        // shooterTargetSpeed = SmartDashboard.getNumber("Shooter Setpoint", shooterTargetSpeed);
        // SmartDashboard.putNumber("Shooter Setpoint", shooterTargetSpeed);

        
        // Convert to RPS
        double targetVelocityRPS = Units.radiansToRotations(shooterTargetSpeed / (ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2)) * powerModifier;

        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putNumber("Target Velocity RPS", targetVelocityRPS);
            SmartDashboard.putNumber("Shooter Motor RPS", shooterMotorLeft.getVelocity().getValueAsDouble());
        }

        // Sets the motor control to target velocity
        shooterMotorLeft.setControl(voltageRequest.withVelocity(targetVelocityRPS).withEnableFOC(true));
        shooterMotorRight.setControl(voltageRequest.withVelocity(targetVelocityRPS).withEnableFOC(true));   
        
        if (!Constants.DISABLE_LOGGING) {
            Logger.recordOutput("Shooter/realVelocity", shooterMotorLeft.getVelocity().getValueAsDouble() * ShooterConstants.SHOOTER_LAUNCH_DIAMETER);
            Logger.recordOutput("Shooter/targetVelocity", shooterTargetSpeed);
        }

        double actualWheelVelocity = shooterMotorLeft.getVelocity().getValueAsDouble() * ShooterConstants.SHOOTER_LAUNCH_DIAMETER;
        
        if (!Constants.DISABLE_SMART_DASHBOARD) {
            SmartDashboard.putNumber("Shooter Speed Error (mps)", shooterTargetSpeed - actualWheelVelocity);
            SmartDashboard.putBoolean("Shooter At Speed", atTargetSpeed());
            SmartDashboard.putBoolean("Shooter Running", shooterTargetSpeed > 0);
        }
        powerModifier = SmartDashboard.getNumber("OPERATOR: Shooter Power Modifier", powerModifier);
        SmartDashboard.putNumber("OPERATOR: Shooter Power Modifier", powerModifier);

        // keep this
        SmartDashboard.putString("WON AUTO?", (HubActive.wonAuto()) ? "WON" : "LOST");
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

    public void setNewCurrentLimit(double stator, double supply) {
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = stator;
        limitConfig.StatorCurrentLimitEnable = true;
        limitConfig.SupplyCurrentLimit = supply;
        limitConfig.SupplyCurrentLimitEnable = true;
        shooterMotorLeft.getConfigurator().apply(limitConfig);
        shooterMotorRight.getConfigurator().apply(limitConfig);
    }

    public double getSubsystemStatorCurrent() {
        return inputs.shooterStatorCurrentLeft + inputs.shooterStatorCurrentRight;
    }

    public double getSubsystemSupplyCurrent() {
        return inputs.shooterSupplyCurrentLeft + inputs.shooterSupplyCurrentRight;
    }

    @Override
    public void updateInputs(){
        inputs.shooterSpeedLeft = Units.rotationsToRadians(shooterMotorLeft.getVelocity().getValueAsDouble()) * ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
        inputs.shooterSpeedRight = Units.rotationsToRadians(shooterMotorRight.getVelocity().getValueAsDouble())* ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2;
        inputs.shooterStatorCurrentLeft = shooterMotorLeft.getStatorCurrent().getValueAsDouble();
        inputs.shooterStatorCurrentRight = shooterMotorRight.getStatorCurrent().getValueAsDouble();
        inputs.shooterSupplyCurrentLeft = shooterMotorLeft.getSupplyCurrent().getValueAsDouble();
        inputs.shooterSupplyCurrentRight = shooterMotorRight.getSupplyCurrent().getValueAsDouble();

        Logger.processInputs("Shooter", inputs);
    }

    public void bumpUpShooterModifier() {
        powerModifier += 0.025;
    }

    public void bumpDownShooterModifier() {
        powerModifier -= 0.025;
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
