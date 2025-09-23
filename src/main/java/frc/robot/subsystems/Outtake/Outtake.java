package frc.robot.subsystems.Outtake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

/**
 * Outtake subsystem responsible for ejecting coral game pieces from the robot.
 * Uses a TalonFX motor for mechanical control and a ColorSensorV3 for proximity
 * detection.
 */
public class Outtake extends SubsystemBase {

    /** TalonFX motor controller that drives the outtake mechanism */
    private TalonFX motor = new TalonFX(IdConstants.OUTTAKE_MOTOR);

    /** Current power level being applied to the motor (-1.0 to 1.0) */
    private double power;

    /**
     * Color sensor that measures coral proximity - detects coral before it reaches
     * the rollers
     */
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    /**
     * Constructor that initializes the outtake subsystem.
     * Configures the motor to be inverted (counter-clockwise positive) and sets
     * brake mode.
     */
    public Outtake() {
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));
    }

    /**
     * Gets the current motor speed/power level.
     * 
     * @return The current power value being applied to the motor (-1.0 to 1.0)
     */
    public double getMotorSpeed() {
        return power;
    }

    /**
     * Periodic method called every robot loop cycle (20ms).
     * Applies the stored power value to the motor controller.
     */
    @Override
    public void periodic() {
        motor.set(power);
    }

    /**
     * Sets the motor power level for the outtake mechanism.
     * 
     * @param power The desired power level (-1.0 to 1.0, negative values for
     *              outtaking)
     */
    public void setMotor(double power) {
        this.power = power;
    }

    /**
     * Activates the outtake mechanism to eject coral at 30% power.
     * Uses negative power value to spin the motor in the outtake direction.
     */
    public void outtake() {
        setMotor(-0.3);
    }

    /**
     * Gets the proximity reading from the color sensor to detect coral presence.
     * If the sensor returns 0 (indicating a potential I2C communication issue),
     * it attempts to reinitialize the sensor and try again.
     * 
     * @return The proximity value from the sensor (higher values indicate closer
     *         objects)
     */
    public int getProximity() {
        int proximity = colorSensor.getProximity();
        if (proximity > 0) {
            return proximity;
        } else {
            // Attempt to fix I2C communication issues by closing and reopening the
            // connection
            I2CJNI.i2CClose(1);
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            return colorSensor.getProximity();
        }
    }

    /**
     * Reverses the outtake mechanism to pull coral back in at 30% power.
     * Uses positive power value to spin the motor in the reverse direction.
     */
    public void reverse() {
        setMotor(0.3);
    }

    /**
     * Checks if a coral game piece is currently loaded in the outtake mechanism.
     * Uses proximity sensor reading with a threshold of 2000 to determine presence.
     * 
     * @return true if coral is detected (proximity > 2000), false otherwise
     */
    public boolean coralLoaded() {
        return getProximity() > 2000;
    }

    /**
     * Determines if coral is currently being ejected from the robot.
     * Checks if the outtake motor is actively running in the ejection direction.
     *
     * @return true if the motor is running in outtake direction (negative power), false otherwise
     */
    public boolean coralEjecting() {
        return power < 0;
    }
}