package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private boolean calibrating = false;
  private Debouncer calibrationDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    double inchExtension = io.getPosition();

    if (calibrating) {
      io.setRightMotor(-0.1);
      io.setLeftMotor(-0.1);
      boolean atHardStop = Math
          .abs((inputs.leftCurrent + inputs.rightCurrent)
              / 2) >= IntakeConstants.CALIBRATING_CURRENT_THRESHOLD;
    }

  }

  /**
   * convert rotations to inches
   * 
   * @param rotations of the motor
   * @return inches of rack travel
   */
  public static double rotationsToInches(double motorRotations) {
    // circumference of the rack pinion
    double circ = 2 * Math.PI * 0.5;
    double pinionRotations = motorRotations / IntakeConstants.GEAR_RATIO;
    double inches = pinionRotations * circ;
    return inches;
  }

  /**
   * convert inches to rotations
   * 
   * @param inches of rack travel
   * @return motor rotations
   */
  public static double inchesToRotations(double inches) {
    double circ = 2 * Math.PI * 0.5;
    double pinionRotations = inches / circ;
    double motorRotations = pinionRotations * IntakeConstants.GEAR_RATIO;
    return motorRotations;
  }

  /**
   * Set the roller speed.
   * 
   * @param speed duty cycle in the range [-1, 1]
   */
  public void spin(double speed) {
    io.setRollerMotor(speed);
  }

  public double getSpeed() {
    return inputs.rollerSetSpeed;
  }

  /**
   * Get the intake extender position
   * 
   * @return inches
   */
  public double getPosition() {
    return inputs.leftPosition;
  }

  /**
   * Start the intake roller spinning.
   */
  public void spinStart() {
    spin(IntakeConstants.SPEED);
  }

  /**
   * Stop the intake roller.
   */
  public void spinStop() {
    spin(0.0);
  }

  /**
   * Reverses the intake roller
   */
  public void spinReverse() {
    spin(-IntakeConstants.SPEED);
  }

  /** Extend the intake the maximum distance. */
  public void extend() {
    io.setPosition(IntakeConstants.MAX_EXTENSION);
  }

  /** Extend to a position that doesn't hit the spindexer */
  public void intermediateExtend() {
    io.setPosition(IntakeConstants.INTERMEDIATE_EXTENSION);
  }

  /** Retract the intake to a safe starting position. */
  public void retract() {
    io.setPosition(IntakeConstants.STOW_EXTENSION);
  }

  /** Goes to the zero position */
  public void zeroPosition() {
    io.setPosition(0.0);
  }

  public void zeroMotors() {
    io.setRawPosition(0.0);
  }

  /**
   * Reclaim all the resources (e.g., motors and other devices).
   * This step is necessary for multiple unit tests to work.
   */
  public void close() {
    io.close();
  }

  /**
   * Starts calibrating by running it backwards
   */
  public void calibrate() {
    setCurrentLimits(IntakeConstants.CALIBRATING_CURRENT_LIMITS);
    calibrating = true;
  }

  /**
   * Stops, zeros, and moves it to retract position
   */
  public void stopCalibrating() {
    zeroMotors();
    setCurrentLimits(IntakeConstants.EXTENDER_CURRENT_LIMITS);
    calibrating = false;
    retract();
  }

  /**
   * sets supply and stator current limits
   * 
   * @param limit the current limit for stator and supply current
   */
  public void setCurrentLimits(double limit) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(limit)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(limit);

    io.setLimits(limits);
  }
}
