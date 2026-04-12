package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterSpeedLeft = 0.0;
    public double shooterSpeedRight = 0.0;
    public double shooterCurrentLeft = 0.0;
    public double shooterCurrentRight = 0.0;
  }

  public void updateInputs(ShooterIOInputs inputs);

  public void setNewCurrentLimit(double newCurrentLimit);

  public void setTargetVelocityRps(double target);
}
