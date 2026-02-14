package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.constants.Climb.ClimbConstants;
import frc.robot.constants.swerve.DriveConstants;

public class LinearClimb extends SubsystemBase{
    private TalonFX motor;

    private static PIDController pid = new PIDController(0.1, 0, 0);
    
    private double gearRatio = ClimbConstants.CLIMB_GEAR_RATIO;

    public LinearClimb() {
        motor = new TalonFX(IdConstants.CLIMB_MOTOR_ID);//, Constants.RIO_CAN);
        pid.setTolerance(0.2);

        motor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits = new CurrentLimitsConfigs();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 5.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 5.0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
        SmartDashboard.putData("Go Up", new InstantCommand(() -> goUp()));
        SmartDashboard.putData("Go Down", new InstantCommand(() -> goDown()));
        SmartDashboard.putData("Climb", new InstantCommand(() -> climb()));
        SmartDashboard.putNumber("Position", getPosition());

        motor.setPosition(0);
    }

    /**
    * set the setpoint for the pid
    * @param setpoint in rotations
    */
    public void setSetpoint(double setpoint) {
        pid.setSetpoint(setpoint);
    }

    public boolean atSetPoint(){
        return pid.atSetpoint();
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public void goUp() {
        setSetpoint(45);
    }

    public void goDown() {
        setSetpoint(ClimbConstants.MIN_HEIGHT);
    }

    public void climb() {
        setSetpoint(ClimbConstants.CLIMB_HEIGHT);
    }

    public void periodic() {
        double power = pid.calculate(motor.getPosition().getValueAsDouble());
        power = MathUtil.clamp(power, -0.2, 0.2);
        motor.set(power);
        SmartDashboard.putNumber("Position", getPosition());
    }
}