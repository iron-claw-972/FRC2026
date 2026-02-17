package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;
import frc.robot.constants.Climb.ClimbConstants;

public class LinearClimb extends SubsystemBase {
    private final TalonFX motor;
    private boolean calibrating = false;
    private int counter = 0;
    private double downPosition = ClimbConstants.OFFSET;
    private double upPosition = 0;
    private double climbPosition = ClimbConstants.CLIMB_OFFSET;

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
        return getPosition() * ClimbConstants.CLIMB_GEAR_RATIO * 2 * Math.PI * ClimbConstants.RADIUS;
    }

    public void goUp() {
        setSetpoint(upPosition);
    }

    public void goDown() {
        setSetpoint(downPosition);
    }

    public void climb() {
        setSetpoint(climbPosition);
    }

    @Override
    public void periodic() {
        if (!calibrating) {
            double power = pid.calculate(motor.getPosition().getValueAsDouble());
            power = MathUtil.clamp(power, ClimbConstants.MIN_POWER, ClimbConstants.MAX_POWER);
            motor.set(power);
        } else {
            if (counter > ClimbConstants.CALIBRATION_COUNTER_LIMIT) {
                stopCalibrating();
            }
            motor.set(ClimbConstants.CALIBRATION_POWER);
            counter += 1;
        }
        SmartDashboard.putNumber("Position", getPosition());
    }

    public void setCurrentLimits(double limit) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits = new CurrentLimitsConfigs();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = limit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = limit;

        motor.getConfigurator().apply(config);
    }

    public void hardstopCalibration() {
        calibrating = true;
        counter = 0;
        setCurrentLimits(ClimbConstants.CALIBRATION_CURRENT);
    }

    public void stopCalibrating() {
        double hardstopPosition = motor.getPosition().getValueAsDouble();
        downPosition = hardstopPosition - ClimbConstants.OFFSET;
        climbPosition = downPosition + ClimbConstants.CLIMB_OFFSET;
        upPosition = hardstopPosition;
        setSetpoint(downPosition);
        calibrating = false;
        counter = 0;
        setCurrentLimits(ClimbConstants.CLIMB_CURRENT);
    }
}
