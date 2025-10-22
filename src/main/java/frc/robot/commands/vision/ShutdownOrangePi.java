package frc.robot.commands.vision;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermissions;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

/**
 * Run the ssh command to shutdown a single Orange Pi.
 * Uses the username and password set in {@link frc.robot.constants.VisionConstants}.
 */
public class ShutdownOrangePi extends Command {
	private String hostname;
	private Process process;

	/**
	 * @param hostname The hostname or IP of the orangepi to shut down.
	 */
	public ShutdownOrangePi(String hostname) {
		assert hostname != null;
		this.hostname = hostname;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void initialize() {
		if (Robot.isSimulation()) {
			// needs to run on an actual roborio because of architecture-specific binaries
			System.out.println("Would shut down OrangePi at " + hostname + " if this was real.");
			return;
		}

		try {
			String initialPath = Filesystem.getDeployDirectory() + "/sshpass";
			Path initalPathPath = Path.of(initialPath);
			String binPath = "/home/lvuser/sshpass2";
			Path binPathPath = Path.of(binPath);
			// copies to be able to get executable permissions on the new binary
			Files.copy(initalPathPath, binPathPath, StandardCopyOption.REPLACE_EXISTING);
			Files.setPosixFilePermissions(binPathPath, PosixFilePermissions.fromString("rwxr-xr-x"));

			String[] commandString = new String[] {
						binPath,
						"-p", "raspberry",
						"ssh",
						"-o", "StrictHostKeyChecking=no",
						VisionConstants.ORANGEPI_USERNAME + "@" + hostname,
						"sudo", "shutdown", "now" 
			};
		
			this.process = Runtime.getRuntime().exec(commandString);
		} catch (Exception e) {
			String message = e.getMessage() == null ? "unknown" : e.getMessage();
			System.out.println("Failed to shutdown OrangePi. Reason: " + e.getClass() + " -- " + message);
		}
	}

	@Override
	public void execute() {
		if (this.process == null) return;

		try {
			InputStream stdout = this.process.getInputStream();
			InputStream stderr = this.process.getErrorStream();

			int remainingStdoutBytes = stdout.available();
			int remainingStderrBytes = stderr.available();

			if (remainingStdoutBytes > 0) {
				byte[] stdoutBytes = stdout.readNBytes(remainingStdoutBytes);
				System.out.println("OPI: " + new String(stdoutBytes, StandardCharsets.UTF_8));
			}

			if (remainingStderrBytes > 0) {
				byte[] stderrBytes = stderr.readNBytes(remainingStderrBytes);
				System.err.println("OPI: " + new String(stderrBytes, StandardCharsets.UTF_8));
			}
		} catch (IOException e) {}
	}

	@Override
	public boolean isFinished() {
		return this.process == null || !this.process.isAlive();
	}

	@Override
	public void end(boolean interrupted) {
		if (this.process == null) return;

		if (this.process.isAlive()) {
			this.process.destroy(); // end the process if we've been interrupted
		} else {
			// only grab exit value if the process has had time to exit
			int exitValue = this.process.exitValue();
			if (exitValue != 0) // abnormal termination
				System.out.println("OrangePi shutdown of " + hostname + " failed with exit code " + exitValue + ".");
			else
				System.out.println("OrangePi shutdown of " + hostname + " succesful.");
		}
	}
}
