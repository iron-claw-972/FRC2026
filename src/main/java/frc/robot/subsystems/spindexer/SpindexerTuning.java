package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpindexerTuning {

  private final TalonFX motor1;
  private final TalonFX motor2;

  private final MotionMagicVelocityTorqueCurrentFOC request =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  private final NeutralOut neutralOut = new NeutralOut();

  private double kS = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;
  private double kP = 1.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private double targetVelocity = 0.0;
  private double maxAccel = 0.0;
  private double maxJerk = 0.0;

  private final ShuffleboardTab tuningTab =
      Shuffleboard.getTab("Indexer Tuning");

  private final GenericEntry kSEntry =
      tuningTab.add("kS", 0.0).getEntry();

  private final GenericEntry kVEntry =
      tuningTab.add("kV", 0.0).getEntry();

  private final GenericEntry kAEntry =
      tuningTab.add("kA", 0.0).getEntry();

  private final GenericEntry kPEntry =
      tuningTab.add("kP", 1.0).getEntry();

  private final GenericEntry kIEntry =
      tuningTab.add("kI", 0.0).getEntry();

  private final GenericEntry kDEntry =
      tuningTab.add("kD", 0.0).getEntry();

  private final GenericEntry maxAccelEntry =
      tuningTab.add("Max Accel", 0.0).getEntry();

  private final GenericEntry maxJerkEntry =
      tuningTab.add("Max Jerk", 0.0).getEntry();

  private final GenericEntry targetVelocityEntry =
      tuningTab.add("Target Velocity RPS", 0.0).getEntry();

  private final GenericEntry enabledEntry =
      tuningTab.add("Enabled", false).getEntry();

  public SpindexerTuning(TalonFX motor1, TalonFX motor2) {
    this.motor1 = motor1;
    this.motor2 = motor2;

    tuningTab
        .add("Update Gains", new InstantCommand(this::applyGains))
        .withPosition(0, 6)
        .withSize(2, 1);
  }

  public void periodic() {

    boolean enabled = enabledEntry.getBoolean(false);

    if (!enabled) {
      motor1.setControl(neutralOut);
      motor2.setControl(neutralOut);
      return;
    }

    kS = kSEntry.getDouble(kS);
    kV = kVEntry.getDouble(kV);
    kA = kAEntry.getDouble(kA);

    kP = kPEntry.getDouble(kP);
    kI = kIEntry.getDouble(kI);
    kD = kDEntry.getDouble(kD);

    maxAccel = maxAccelEntry.getDouble(maxAccel);
    maxJerk = maxJerkEntry.getDouble(maxJerk);

    // low = 6-7 RPS high = 60-67 RPS https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/manual-pid-tuning.html#flywheel-tuning-with-torquecurrentfoc
    targetVelocity =
        targetVelocityEntry.getDouble(targetVelocity);

    motor1.setControl(
        request.withVelocity(targetVelocity));

    motor2.setControl(
        request.withVelocity(targetVelocity));

    Logger.recordOutput(
        "Indexer Tuning/Target Velocity RPS",
        targetVelocity);

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

    Logger.recordOutput(
        "Indexer Tuning/Max Accel",
        maxAccel);

    Logger.recordOutput(
        "Indexer Tuning/Max Jerk",
        maxJerk);

    Logger.recordOutput(
        "Indexer Tuning/Actual Acceleration r/s^2",
        motor1.getAcceleration().getValueAsDouble());

    Logger.recordOutput(
        "Indexer Tuning/Acceleration Cap",
        motor1.isSafetyEnabled());
  }

  private void applyGains() {
    System.out.println("UPDATING GAINS");

    TalonFXConfiguration config = new TalonFXConfiguration();

    // acceleration = rotations/sec^2
    // jerk = rotations/sec^3
    MotionMagicConfigs motionMagic = config.MotionMagic;

    motionMagic.MotionMagicAcceleration = maxAccel;
    motionMagic.MotionMagicJerk = maxJerk;  //0 means no lim

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