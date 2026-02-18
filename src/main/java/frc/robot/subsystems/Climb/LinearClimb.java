package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class LinearClimb extends SubsystemBase {
    private final TalonFX motor;
    private boolean calibrating = false;
    private double downPosition = ClimbConstants.BOTTOM_POSITION;
    private double upPosition = ClimbConstants.UP_POSITION;
    private double climbPosition = ClimbConstants.CLIMB_POSITION;

    private final PIDController pid = new PIDController(
            ClimbConstants.PID_P,
            ClimbConstants.PID_I,
            ClimbConstants.PID_D);

    public LinearClimb() {
        motor = new TalonFX(IdConstants.CLIMB_MOTOR_ID);
        pid.setTolerance(ClimbConstants.PID_TOLERANCE);

        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);

        setCurrentLimits(ClimbConstants.CALIBRATION_CURRENT);

        SmartDashboard.putData("Go Up", new InstantCommand(() -> goUp()));
        SmartDashboard.putData("Go Down", new InstantCommand(() -> goDown()));
        SmartDashboard.putData("Climb", new InstantCommand(() -> climb()));
        SmartDashboard.putData("Hardstop Calibrate", new InstantCommand(() -> hardstopCalibration()));
        SmartDashboard.putData("Stop Calibrating", new InstantCommand(() -> stopCalibrating()));
        SmartDashboard.putNumber("Position", getPosition());

        motor.setPosition(0);

        // calibrate on startup to find hardstop
        hardstopCalibration();
    }

    /**
     * set the setpoint for the pid
     * 
     * @param setpoint in rotations
     */
    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
    }
    /**
     * @return when the position is within 0.2 rotations
     */
    public boolean atSetPoint() {
        return pid.atSetpoint();
    }

    /**
     * Returns the current position of the climb motor.
     * 
     * @return Position in motor rotations. Positive values move the climb mechanism
     *         UP (toward the hardstop). Higher values = higher physical position.
     *         Use {@link #getAsMeters()} for linear distance in meters.
     */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Returns the climb position converted to linear distance in meters.
     * This is useful for debugging and logging.
     * 
     * @return Linear position in meters, calculated as:
     *         rotations * gearRatio * 2 * PI * radius
     */
    public double getAsMeters() {
        return getPosition() * ClimbConstants.CLIMB_GEAR_RATIO * 2 * Math.PI * ClimbConstants.WHEEL_RADIUS;
    }
    /**
     * goes to the up position
     */
    public void goUp() {
        setSetpoint((Units.radiansToRotations(upPosition / ClimbConstants.WHEEL_RADIUS)) * ClimbConstants.CLIMB_GEAR_RATIO);
    }
    /**
     * goes to the down position
     */
    public void goDown() {
        setSetpoint((Units.radiansToRotations(downPosition / ClimbConstants.WHEEL_RADIUS)) * ClimbConstants.CLIMB_GEAR_RATIO);
    }
    /**
     * goes to the climb position
     */
    public void climb() {
        setSetpoint((Units.radiansToRotations(climbPosition / ClimbConstants.WHEEL_RADIUS)) * ClimbConstants.CLIMB_GEAR_RATIO);
    }

    @Override
    public void periodic() {
        // if it is not calibrating, do normal stuff
        if (!calibrating) {
            double power = pid.calculate(motor.getPosition().getValueAsDouble());
            power = MathUtil.clamp(power, ClimbConstants.MIN_POWER, ClimbConstants.MAX_POWER);
            motor.set(power);
        } else {
            motor.set(ClimbConstants.CALIBRATION_POWER);
        }
        SmartDashboard.putNumber("Position", getPosition());
    }
    /**
     * sets supply and stator current limits
     * @param limit the current limit for stator and supply current
     */
    public void setCurrentLimits(double limit) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits = new CurrentLimitsConfigs();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = limit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = limit;

        motor.getConfigurator().apply(config);
    }
    /**
     * Sets the motor to a slow speed until it hits the hard stop
     */
    public void hardstopCalibration() {
        calibrating = true;
        setCurrentLimits(ClimbConstants.CALIBRATION_CURRENT);
    }
    /**
     * stops calibration and sets current limits to normal. 
     */
    public void stopCalibrating() {
        motor.setPosition(downPosition);
        calibrating = false;
        setCurrentLimits(ClimbConstants.CLIMB_CURRENT);
    }
}
