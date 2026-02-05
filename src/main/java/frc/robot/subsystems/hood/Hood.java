package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.IdConstants;

public class Hood extends SubsystemBase {
    final private TalonFX motor;
    private double position;
    private double velocity;
    double power;

    private double setpoint;

    private PIDController pid = new PIDController(0.01, 0.0, 0.0);

    public Hood() {
        motor = new TalonFX(IdConstants.HOOD_MOTOR_ID, Constants.SUBSYSTEM_CANIVORE_CAN);

        pid.setTolerance(Units.degreesToRadians(3));

        motor.setPosition(Units.degreesToRotations(HoodConstants.START_ANGLE) * HoodConstants.HOOD_GEAR_RATIO);
        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();

        limitConfig.StatorCurrentLimit = 40; // 120
        limitConfig.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(limitConfig);

        // config.Slot0.kS = 0.1; // Static friction compensation (should be >0 if friction exists)
        // config.Slot0.kG = 0.25; // Gravity compensation
        // config.Slot0.kV = 0.12; // Velocity gain: 1 rps -> 0.12V
        // config.Slot0.kA = 0; // Acceleration gain: 1 rps² -> 0V (should be tuned if acceleration matters)

        // config.Slot0.kP = Units.radiansToRotations(3.0 * 12); // If position error is 2.5 rotations, apply 12V (0.5 *
        //                                                       // 2.5 * 12V)
        // config.Slot0.kI = Units.radiansToRotations(0.00); // Integral term (usually left at 0 for MotionMagic)
        // config.Slot0.kD = Units.radiansToRotations(0.00 * 12); // Derivative term (used to dampen oscillations)

        // motor.getConfigurator().apply(config);

        // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MAX_ANGLE)
        //         * HoodConstants.HOOD_GEAR_RATIO;

        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(HoodConstants.MIN_ANGLE)
        //         * HoodConstants.HOOD_GEAR_RATIO;

        SmartDashboard.putData("Set 45 degrees", new InstantCommand(() -> setSetpoint(45.0)));
        SmartDashboard.putData("Recalibrate Hood", new InstantCommand(() -> resetDueToSlippingError()));

        SmartDashboard.putData("Move to max angle", new InstantCommand(() -> setSetpoint(HoodConstants.MAX_ANGLE)));
        SmartDashboard.putData("Move to min angle", new InstantCommand(() -> setSetpoint(HoodConstants.MIN_ANGLE)));
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = Units.degreesToRotations(MathUtil.clamp(setpoint, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE)) * HoodConstants.HOOD_GEAR_RATIO;
    }

    public double getPosition() {
        return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble())
                / HoodConstants.HOOD_GEAR_RATIO;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getVelocity() {
        return velocity / HoodConstants.HOOD_GEAR_RATIO;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - setpoint) < 3.0;
    }

    @Override
    public void periodic() {
        position = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) / HoodConstants.HOOD_GEAR_RATIO;
        velocity = Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble() * 60);
        
        // setSetpoint(SmartDashboard.getNumber("hood setpoint", Units.degreesToRadians(getSetpoint())));

        SmartDashboard.putNumber("Hood Position", position);
        SmartDashboard.putNumber("hood setpoint", getSetpoint());

        Logger.recordOutput("HoodPitch", getPosition());

        motor.set(pid.calculate(motor.getPosition().getValueAsDouble(), setpoint));

    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }



    // Intended to be used for the slipping of the bands that are on the gears
    public void resetDueToSlippingError() {
        long startTime = System.currentTimeMillis();
        long timeout = 5000;

        while (motor.getSupplyCurrent().getValueAsDouble() < HoodConstants.CURRENT_SPIKE_THRESHHOLD) {
            if (System.currentTimeMillis() - startTime > timeout) {
                System.err.println("Hood reset timeout: current spike threshold not reached");
                break;
            }
            motor.setVoltage(4);
        }
        position = HoodConstants.START_ANGLE;
    }

}
