package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpindexerTuning {

  private final TalonFX motor1;
  private final TalonFX motor2;

  private final MotionMagicVelocityTorqueCurrentFOC request = new MotionMagicVelocityTorqueCurrentFOC(0);

  private final NeutralOut neutralOut = new NeutralOut();

  private double kS = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private double targetVelocity = 0.0;
  private double maxAccel = 0.0;
  private double maxJerk = 0.0;

  public SpindexerTuning(TalonFX motor1, TalonFX motor2) {
    this.motor1 = motor1;
    this.motor2 = motor2;

    SmartDashboard.putNumber("Indexer Tuning/kS", 0.0);
    SmartDashboard.putNumber("Indexer Tuning/kV", 0.0);
    SmartDashboard.putNumber("Indexer Tuning/kA", 0.0);
    SmartDashboard.putNumber("Indexer Tuning/kP", 1.0);
    SmartDashboard.putNumber("Indexer Tuning/kI", 0.0);
    SmartDashboard.putNumber("Indexer Tuning/kD", 0.0);
    SmartDashboard.putNumber("Indexer Tuning/maxAccel", 0.0);
    SmartDashboard.putNumber("Indexer Tuning/maxJerk", 0.0);

    SmartDashboard.putNumber(
        "Indexer Tuning/Target Velocity RPS",
        0.0);

    SmartDashboard.putBoolean(
        "Indexer Tuning/Enabled",
        false);

    SmartDashboard.putData("Indexer Tuning/update", new InstantCommand(() -> applyGains()));
  }

  public void periodic() {

    boolean enabled = SmartDashboard.getBoolean(
        "Indexer Tuning/Enabled",
        false);

    if (!enabled) {
      motor1.setControl(neutralOut);
      return;
    }

    kS = SmartDashboard.getNumber(
        "Indexer Tuning/kS",
        kS);

    kV = SmartDashboard.getNumber(
        "Indexer Tuning/kV",
        kV);

    kA = SmartDashboard.getNumber(
        "Indexer Tuning/kA",
        kA);

    kP = SmartDashboard.getNumber(
        "Indexer Tuning/kP",
        kP);

    kI = SmartDashboard.getNumber(
        "Indexer Tuning/kI",
        kI);

    kD = SmartDashboard.getNumber(
        "Indexer Tuning/kD",
        kD);

    maxAccel = SmartDashboard.getNumber(
        "Indexer Tuning/maxAccel",
        maxAccel);

    maxJerk = SmartDashboard.getNumber(
        "Indexer Tuning/maxJerk",
        maxJerk);

    // low = 6-7 rps, high: 60-67 rps for tuning: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/manual-pid-tuning.html#flywheel-tuning-with-torquecurrentfoc
    targetVelocity = SmartDashboard.getNumber(
        "Indexer Tuning/Target Velocity RPS",
       targetVelocity);

    // applyGains();

    motor1.setControl(
        request.withVelocity(targetVelocity));
    motor2.setControl(
        request.withVelocity(targetVelocity));

    Logger.recordOutput("Indexer Tuning/Target Velocity RPS", targetVelocity);

    Logger.recordOutput(
        "Indexer Tuning/Actual Velocity RPS",
        motor1.getVelocity().getValueAsDouble());

    Logger.recordOutput(
        "Indexer Tuning/Velocity Error RPS",
        motor1.getClosedLoopError().getValueAsDouble());

    Logger.recordOutput(
        "Indexer Tuning/Torque Current Amps",
        motor1.getTorqueCurrent().getValueAsDouble());

    Logger.recordOutput(
        "Indexer Tuning/Closed Loop Reference",
        motor1.getClosedLoopReference().getValueAsDouble());

    Logger.recordOutput("Indexer Tuning/Max Accel", maxAccel);
    Logger.recordOutput("Indexer Tuning/Max Jerk", maxJerk);

    Logger.recordOutput("Indexer Tuning/Actual Acceleration r/s^2", motor1.getAcceleration().getValueAsDouble());

    Logger.recordOutput("Indexer Tuning/Acceleration Cap", motor1.isSafetyEnabled());
  }

  private void applyGains() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    // acceleration = rotations/sec^2
    // jerk = rotations/sec^3
    MotionMagicConfigs motionMagic = config.MotionMagic;

    motionMagic.MotionMagicAcceleration = maxAccel; // 0 means no lim
    motionMagic.MotionMagicJerk = maxJerk;

    Slot0Configs slot0 = config.Slot0;

    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;

    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }
}
