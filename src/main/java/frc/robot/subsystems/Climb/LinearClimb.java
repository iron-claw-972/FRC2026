package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;

/**
 * Climber subsystem
 */
public class LinearClimb extends SubsystemBase implements LinearClimbIO{
    /** climber motor */
    private final TalonFX motor;
    /** whether the subsysgtem is calibrating */
    private boolean calibrating = false;

    /** should the subsystem perform calibration automatically */
    private boolean calibrateOnStartUp = false;

    private double MAX_POWER = 0.2;

    private Debouncer calibrationDebouncer = new Debouncer(0.5, DebounceType.kRising);

    // logging information
    private LinearClimbIOInputsAutoLogged inputs = new LinearClimbIOInputsAutoLogged();

    private final PIDController pid = new PIDController(
            ClimbConstants.PID_P,
            ClimbConstants.PID_I,
            ClimbConstants.PID_D);

    public LinearClimb() {
        motor = new TalonFX(IdConstants.CLIMB_MOTOR_ID, Constants.CANIVORE_SUB);
        pid.setTolerance(ClimbConstants.PID_TOLERANCE);

        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);

        setCurrentLimits(ClimbConstants.CALIBRATION_CURRENT);

        SmartDashboard.putData("Go Up", new InstantCommand(() -> goUp()));
        SmartDashboard.putData("Go Down", new InstantCommand(() -> retract()));
        SmartDashboard.putData("Climb", new InstantCommand(() -> climbPosition()));

        motor.setPosition(0);

        // calibrate on startup to find hardstop
        if(calibrateOnStartUp){
            hardstopCalibration();
        }
    }

    /**
     * set the setpoint for the pid
     * 
     * @param setpoint in rotations
     */
    public void setSetpoint(double setpoint) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
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
        return inputs.positionMeters;
    }

    /**
     * goes to the up position
     */
    public void goUp() {
        MAX_POWER = 0.8;
        setSetpoint(metersToRotations(ClimbConstants.UP_POSITION));
    }

    /**
     * goes to the down position
     */
    public void retract() {
        MAX_POWER = 0.2;
        setSetpoint(metersToRotations(ClimbConstants.BOTTOM_POSITION));
    }

    /**
     * goes to the climb position
     */
    public void climbPosition() {
        MAX_POWER = 0.8;
        setSetpoint(metersToRotations(ClimbConstants.CLIMB_POSITION));
    }

    @Override
    public void periodic() {
        double power = pid.calculate(motor.getPosition().getValueAsDouble());
        // if it is not calibrating, do normal stuff
        if (!calibrating) {
            power = MathUtil.clamp(power, -MAX_POWER, MAX_POWER);
        } else{
            power = ClimbConstants.CALIBRATION_POWER;
            boolean atHardStop = Math.abs(motor.getStatorCurrent().getValueAsDouble()) >= ClimbConstants.CALIBRATION_CURRENT_THRESHOLD;
            if(calibrationDebouncer.calculate(atHardStop)){
                stopCalibrating();
            }
        }
        motor.set(power);
        
        Logger.recordOutput("LinearClimb/setpointMeters", Units.rotationsToRadians(pid.getSetpoint()) * ClimbConstants.WHEEL_RADIUS / ClimbConstants.CLIMB_GEAR_RATIO);
        updateInputs();
        Logger.processInputs("LinearClimb", inputs);
    }
    /**
     * converts motor rotations to meters
     * @param motorRotations
     * @return
     */
    public double rotationsToMeters(double motorRotations){
        double circ = 2 * Math.PI * ClimbConstants.WHEEL_RADIUS;
        double meters = motorRotations / ClimbConstants.CLIMB_GEAR_RATIO * circ;
        return meters;
    }

    /**
     * converts meters to motor rotations
     * @param meters
     * @return
     */
    public double metersToRotations(double meters){
        double circ = 2 * Math.PI * ClimbConstants.WHEEL_RADIUS;
        double motorRotations = meters / circ * ClimbConstants.CLIMB_GEAR_RATIO;
        return motorRotations;
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
        motor.setPosition(metersToRotations(ClimbConstants.BOTTOM_POSITION));
        calibrating = false;
        setCurrentLimits(ClimbConstants.CLIMB_CURRENT);
        setSetpoint(metersToRotations(ClimbConstants.BOTTOM_POSITION));
    }

    @Override
    public void updateInputs() {
        inputs.positionMeters = Units.rotationsToRadians(motor.getPosition().getValueAsDouble()) * ClimbConstants.WHEEL_RADIUS / ClimbConstants.CLIMB_GEAR_RATIO;
        inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    }
}
