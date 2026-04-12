package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Shooter extends SubsystemBase {
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    LoggedNetworkNumber powerModifier = new LoggedNetworkNumber("OPERATOR: Shooter Power Modifier", 1.05);
    ShooterIO io;

    
    // Goal Velocity / Double theCircumfrence
    private double shooterTargetSpeed = 0;

    public Shooter(ShooterIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // shooterTargetSpeed = SmartDashboard.getNumber("Shooter Setpoint", shooterTargetSpeed);
        // SmartDashboard.putNumber("Shooter Setpoint", shooterTargetSpeed);

        
        // Convert to RPS
        double targetVelocityRPS = Units.radiansToRotations(shooterTargetSpeed / (ShooterConstants.SHOOTER_LAUNCH_DIAMETER/2)) * powerModifier.get();


        // Sets the motor control to target velocity
        io.setTargetVelocityRps(targetVelocityRPS);
        
        if (!Constants.DISABLE_LOGGING) {
            Logger.recordOutput("Shooter/realVelocity", inputs.shooterSpeedLeft);
            Logger.recordOutput("Shooter/targetVelocity", shooterTargetSpeed);
        }
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

    public void bumpUpShooterModifier() {
        powerModifier.set(powerModifier.get() + 0.025);
    }

    public void bumpDownShooterModifier() {
        powerModifier.set(powerModifier.get() - 0.025);
    }

    /**
     * @return Whether the shooter is at the target speed with tolerance of 1 m/s
     */
    public boolean atTargetSpeed(){
        return Math.abs(getShooterVelocity() - shooterTargetSpeed) < 1.0;
    }

    public void setNewCurrentLimits(double limit) {
        io.setNewCurrentLimit(limit);
    }

    /**
     * @return Gets the target velocity in m/s
     */
    @AutoLogOutput(key="Shooter/TargetSpeed")
    public double getTargetVelocity(){
        return shooterTargetSpeed;
    }
}
