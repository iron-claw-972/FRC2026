package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Climb extends SubsystemBase {

	/** Sets climb to specific angle (degrees) */
	public abstract void setAngle(double angle);

	/** Gets current climb angle (degrees) */
	public abstract double getAngle();

	/** Extends climb to extended position. */
	public abstract void extend();

	/** Moves climb to climb position. */
	public abstract void climb();

	/** Moves the climb to stowed position. */
	public abstract void stow();

	/** Resets the climb */
	public abstract void reset(boolean resetting);

	/** Gets current draw of climb motor in amps. */
	public abstract double getCurrent();

	@Override
	public void periodic() {
	
	}

	@Override
	public void simulationPeriodic() {

    }

}
