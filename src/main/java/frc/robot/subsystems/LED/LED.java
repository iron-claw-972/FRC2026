package frc.robot.subsystems.LED;

import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;

public class LED extends SubsystemBase {

	private CANdle candle;
	public static final int stripLength = 67;

	// Constructor
	public LED() {
		candle = new CANdle(IdConstants.CANDLE_ID);
		CANdleConfigurator configurator = candle.getConfigurator();

		LEDConfigs ledConf = new LEDConfigs()
				.withStripType(StripTypeValue.GRB)
				.withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning)
				.withBrightnessScalar(1);

		CANdleFeaturesConfigs featureConf = new CANdleFeaturesConfigs()
				.withEnable5VRail(Enable5VRailValue.Enabled) // Turns off LEDs
				.withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled)
				.withVBatOutputMode(VBatOutputModeValue.On);

		configurator.apply(featureConf);
		configurator.apply(ledConf);

		System.out.println("CANdle features: " + featureConf + ", LED config: " + ledConf);
	}

	@Override
	public void periodic() {
	}

	/**
	 * Sets the color of all the LEDs.
	 *
	 * @param red   Red value (0-255)
	 * @param green Green value (0-255)
	 * @param blue  Blue value (0-255)
	 */
	public void setLEDs(int red, int green, int blue) {
		setSection(red, green, blue, 0, stripLength);
	}

	/**
	 * Sets an animation for the LEDs.
	 *
	 * @param animation The animation object (e.g., RainbowAnimation,
	 *                  StrobeAnimation, etc.)
	 */
	public void animate(ControlRequest animation) {
		candle.setControl(animation);
	}

	/**
	 * Sets the color of a specific section of LEDs.
	 *
	 * @param r     Red value (0-255)
	 * @param g     Green value (0-255)
	 * @param b     Blue value (0-255)
	 * @param start Start index of the section
	 * @param end   End index of the section
	 */
	public void setSection(int r, int g, int b, int start, int end) {
		SolidColor request = new SolidColor(start, end)
				.withColor(new RGBWColor(r, g, b));
		candle.setControl(request);
	}

	/**
	 * Creates an alternating pattern of two colors across the LEDs.
	 *
	 * @param r1     Red value of the first color (0-255)
	 * @param g1     Green value of the first color (0-255)
	 * @param b1     Blue value of the first color (0-255)
	 * @param r2     Red value of the second color (0-255)
	 * @param g2     Green value of the second color (0-255)
	 * @param b2     Blue value of the second color (0-255)
	 * @param size   Size of each color block
	 * @param offset Offset for the starting position of the pattern
	 * @param total  Total number of LEDs
	 */
	public void alternate(int r1, int g1, int b1, int r2, int g2, int b2, int size, int offset, int total) {
		for (int i = -offset; i < total; i += size) {
			boolean color2 = ((i - offset) / size) % 2 == 0;
			if (color2) {
				setSection(r2, g2, b2, i, i + size - 1);
			} else {
				setSection(r1, g1, b1, i, i + size - 1);
			}
		}
	}
}
