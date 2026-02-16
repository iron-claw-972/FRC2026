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
    private TalonFX motor;
    private boolean calibrating = false;
    double counter = 0;
    double downPosition = ClimbConstants.OFFSET;
    double upPosition = 0;
    double climbPosition = ClimbConstants.CLIMB_OFFSET;

    private static PIDController pid = new PIDController(0.1, 0, 0);

    private double gearRatio = ClimbConstants.CLIMB_GEAR_RATIO;

    public LinearClimb() {
        motor = new TalonFX(IdConstants.CLIMB_MOTOR_ID);// , Constants.RIO_CAN);
        pid.setTolerance(0.2);

        motor.setNeutralMode(NeutralModeValue.Brake);

        setCurrentLimits(5.0);

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

    public void periodic() {
        if (calibrating == false) {
            double power = pid.calculate(motor.getPosition().getValueAsDouble());
            power = MathUtil.clamp(power, -0.2, 0.2);
            motor.set(power);
        } else {
            if (counter > 250) {
                stopCalibrating();
            }
            motor.set(0.15);
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

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
    }

    public void hardstopCalibration() {
        calibrating = true;
        counter = 0;
        setCurrentLimits(ClimbConstants.WEAK_CURRENT);
    }

    public void stopCalibrating() {
        downPosition = motor.getPosition().getValueAsDouble() - 1.0;
        upPosition = downPosition - ClimbConstants.OFFSET;
        climbPosition = upPosition + ClimbConstants.CLIMB_OFFSET;
        setSetpoint(downPosition);
        calibrating = false;
        counter = 0;
        setCurrentLimits(ClimbConstants.STRONG_CURRENT);
    }
}