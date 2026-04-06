package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.Spindexer;

public class SpindexerDeadline extends Command {
  private Spindexer spindexer;
  private Timer airTimer = new Timer();

  private final double AIR_SPINNING_MAX_TIME = .5;

  public SpindexerDeadline(Spindexer spindexer) {
    this.spindexer = spindexer;
  }

  @Override
  public void initialize() {
    airTimer.reset();
    airTimer.start();
  }

  @Override
  public boolean isFinished() {
    return spindexer.spinningAir() && airTimer.hasElapsed(AIR_SPINNING_MAX_TIME);
  }
}
