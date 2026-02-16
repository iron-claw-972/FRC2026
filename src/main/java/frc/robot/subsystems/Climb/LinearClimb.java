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
    private double counter = 0;
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

        setCurrentLimits(ClimbConstants.DEFAULT_CURRENT_LIMIT);

        SmartDashboard.putData("Go Up", new InstantCommand(() -> goUp()));
        SmartDashboard.putData("Go Down", new InstantCommand(() -> goDown()));
        SmartDashboard.putData("Climb", new InstantCommand(() -> climb()));
        SmartDashboard.putData("Hardstop Calibrate", new InstantCommand(() -> hardstopCalibration()));
        SmartDashboard.putData("Stop Calibrating", new InstantCommand(() -> stopCalibrating()));
        SmartDashboard.putNumber("Position", getPosition());

        motor.setPosition(0);
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

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
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
        setCurrentLimits(ClimbConstants.WEAK_CURRENT);
    }

    public void stopCalibrating() {
        downPosition = motor.getPosition().getValueAsDouble() - ClimbConstants.CALIBRATION_POSITION_OFFSET;
        upPosition = downPosition - ClimbConstants.OFFSET;
        climbPosition = upPosition + ClimbConstants.CLIMB_OFFSET;
        setSetpoint(downPosition);
        calibrating = false;
        counter = 0;
        setCurrentLimits(ClimbConstants.STRONG_CURRENT);
    }
}
