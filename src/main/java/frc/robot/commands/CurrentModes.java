
package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.BrownOut.BrownOutConstants;
import frc.robot.util.BrownOut.BrownOutLevel;

public class CurrentModes extends Command {
  private Shooter shooter;
  private Spindexer spindexer;
  private Turret turret;
  private Intake intake;
  private Hood hood;
  private Drivetrain drivetrain;

  private Modes previous_mode;
  private Modes mode = Modes.NORMAL;

  public enum Modes {
    NORMAL,
    SHOOTING_FOCUS,
    PURE_DRIVETRAIN
  }

  public record ModeCurrents(double shooter, double spindexer, double turret, double intake, double hood, double steer,
      double drive) {
  }

  public static final HashMap<Modes, ModeCurrents> MODE_MAP = new HashMap<>();
  static {
    MODE_MAP.put(Modes.NORMAL, new ModeCurrents(ShooterConstants.SHOOTER_CURRENT_LIMIT,
      SpindexerConstants.currentLimit * 1.0,
      TurretConstants.NORMAL_CURRENT_LIMIT * 1.0,
      IntakeConstants.NORMAL_CURRENT_LIMIT * 1.0,
      HoodConstants.NORMAL_CURRENT_LIMIT * 1.0,
      DriveConstants.STEER_PEAK_CURRENT_LIMIT * 1.0,
      DriveConstants.DRIVE_PEAK_CURRENT_LIMIT * 1.0
    ));

    MODE_MAP.put(Modes.SHOOTING_FOCUS, new ModeCurrents(ShooterConstants.SHOOTER_CURRENT_LIMIT,
      SpindexerConstants.currentLimit * 2.0,
      TurretConstants.NORMAL_CURRENT_LIMIT * 1.0,
      IntakeConstants.NORMAL_CURRENT_LIMIT * 1.0,
      HoodConstants.NORMAL_CURRENT_LIMIT * 1.0, //TODO tune
      DriveConstants.STEER_PEAK_CURRENT_LIMIT * 0.6,
      DriveConstants.DRIVE_PEAK_CURRENT_LIMIT * 0.2
    ));

    MODE_MAP.put(Modes.PURE_DRIVETRAIN, new ModeCurrents(ShooterConstants.SHOOTER_CURRENT_LIMIT,
      SpindexerConstants.currentLimit * 0.0,
      TurretConstants.NORMAL_CURRENT_LIMIT * 0.0,
      IntakeConstants.NORMAL_CURRENT_LIMIT * 0.0,
      HoodConstants.NORMAL_CURRENT_LIMIT * 0.0,
      DriveConstants.STEER_PEAK_CURRENT_LIMIT * 2.0,
      DriveConstants.DRIVE_PEAK_CURRENT_LIMIT * 2.0
    ));
  }

  public CurrentModes(Shooter shooter, Spindexer spindexer, Turret turret, Intake intake, Hood hood,
      Drivetrain drivetrain) {
    this.shooter = shooter;
    this.spindexer = spindexer;
    this.turret = turret;
    this.intake = intake;
    this.hood = hood;
    this.drivetrain = drivetrain;
  }

  @Override
  public void execute() {
    if (mode == previous_mode) {
      applyLevel(mode);
    }
    previous_mode = mode;

  }

  public void applyLevel(Modes mode) {
    double shooterCurrent = MODE_MAP.get(mode).shooter();
    double turretCurrent = MODE_MAP.get(mode).turret();
    double hoodCurrent = MODE_MAP.get(mode).hood();
    double spindexerCurrent = MODE_MAP.get(mode).spindexer();
    double intakeCurrent = MODE_MAP.get(mode).intake();
    double steerCurrent = MODE_MAP.get(mode).steer();
    double driveCurrent = MODE_MAP.get(mode).drive();

    // apply them / set them
    shooter.setNewCurrentLimit(shooterCurrent);
    turret.setCurrentLimits(turretCurrent);
    hood.setCurrentLimits(hoodCurrent);
    spindexer.setNewCurrentLimit(spindexerCurrent);
    intake.setCurrentLimits(intakeCurrent);
    drivetrain.applyNewModuleCurrents(steerCurrent, driveCurrent);

  }

  public void setMode(Modes mode) {
    this.mode = mode;
  }

  @Override
  public void end(boolean interrupted) {
    // Nothing
    applyLevel(Modes.NORMAL); // disable
  }
}
