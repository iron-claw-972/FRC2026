package frc.robot.subsystems.LED;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IdConstants;
import frc.robot.util.HubActive;

public class LED2 extends SubsystemBase {

	private CANdle candle;
	public static final int stripLength = 67;

	/// Hz
	public static final int FLASH_RATE = 4;

	private Color color;
	
	public LED2() {
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

		var alliance = DriverStation.getAlliance();
		if (alliance.isEmpty()) {
			color = Color.kWhite;
		} else if (alliance.get() == Alliance.Red) {
			color = Color.kRed;
		} else if (alliance.get() == Alliance.Blue) {
			color = Color.kBlue;
		} else {
			color = Color.kWhite;
		}

		setStatic();

		System.out.println("CANdle features: " + featureConf + ", LED config: " + ledConf);

		SmartDashboard.putData("LED Strobe", new InstantCommand(() -> setStrobe()));
		SmartDashboard.putData("LED Static", new InstantCommand(() -> setStatic()));
	}

	private boolean flippy = true;

	@Override
	public void periodic() {
		if (underSecsToFlip(5.0) && flippy) {
			setStrobe();
			flippy = false;
		} else if (!underSecsToFlip(5.0) && !flippy) {
			setStatic();
			flippy = true;
		}
	}

	private void setStrobe() {
		candle.setControl(new StrobeAnimation(8, 8 + stripLength).withFrameRate(FLASH_RATE).withColor(new RGBWColor(color)));
	}

	private void setStatic() {
		candle.setControl(new SolidColor(8, 8 + stripLength).withColor(new RGBWColor(color)));
	}

	private boolean underSecsToFlip(double secs) {
		Optional<Double> timeToActive = HubActive.timeToActive();
		Optional<Double> timeToInactive = HubActive.timeToInactive();

		if (timeToActive.isEmpty() && timeToInactive.isEmpty()) {
			return false;
		} else if (timeToActive.isPresent()) {
			return (timeToActive.get() <= secs);

		} else if (timeToInactive.isPresent()) {
			return (timeToInactive.get() <= secs);
		} else {
			return false;
		}
	}
}
