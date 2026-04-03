package frc.robot.subsystems.LED;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
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
import frc.robot.constants.Constants;
import frc.robot.constants.IdConstants;
import frc.robot.util.HubActive;

public class LED2 extends SubsystemBase {

	private CANdle candle;
	public static final int stripLength = 67;

	/// Hz
	public static final int FLASH_RATE = 4;

	private Color color;
	
	public LED2() {
		candle = new CANdle(IdConstants.CANDLE_ID, Constants.RIO_CAN);
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

		setColor();

		candle.clearAllAnimations();
		lightsOff();

		System.out.println("CANdle features: " + featureConf + ", LED config: " + ledConf);

		SmartDashboard.putData("LED Disable", new InstantCommand(() -> {forceOff = true; lightsOff();}).ignoringDisable(true));
		SmartDashboard.putData("LED Enable", new InstantCommand(() -> forceOff = false).ignoringDisable(true));
		SmartDashboard.putData("LED Strobe", new InstantCommand(() -> setStrobe()).ignoringDisable(true));
		SmartDashboard.putData("LED Static", new InstantCommand(() -> setStatic()).ignoringDisable(true));
		SmartDashboard.putData("LED Fire", new InstantCommand(() -> setFire()).ignoringDisable(true));
		SmartDashboard.putData("LED Rainbow", new InstantCommand(() -> setRainbow()).ignoringDisable(true));
		SmartDashboard.putData("LED Fade", new InstantCommand(() -> setRgbFadeAnimation()).ignoringDisable(true));
		SmartDashboard.putData("LED Twinkle", new InstantCommand(() -> setTwinkle()).ignoringDisable(true));

		SmartDashboard.putData("LED Color Team Reset", new InstantCommand(() -> setColor()).ignoringDisable(true));
	}

	public void setColor() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isEmpty()) {
			color = Color.kOrangeRed;
		} else if (alliance.get() == Alliance.Red) {
			color = Color.kRed;
		} else if (alliance.get() == Alliance.Blue) {
			color = Color.kBlue;
		} else {
			color = Color.kOrangeRed;
		}
	}

	private enum State { OFF, ON, AUTO, SLOW, FAST };

	private State lastState = State.OFF;
	private boolean forceOff = false;

	@Override
	public void periodic() {
		State targetState = State.ON;
		if (underSecsToFlip(5)) targetState = State.SLOW;
		if (underSecsToFlip(1)) targetState = State.FAST;
		if (DriverStation.isAutonomous()) targetState = State.AUTO;
		if (forceOff) targetState = State.OFF;

		if (targetState != lastState) {
			switch (targetState) {
				case OFF: lightsOff(); break;
				case ON: setStatic(); break;
				case AUTO: setFire(); break;
				case SLOW: setStrobe(); break;
				case FAST: setFastStrobe(); break;
			}
			lastState = targetState;
		}
	}

	public void setFire() {
		candle.clearAllAnimations();
		candle.setControl(new FireAnimation(8, 8 + stripLength));
	}

	public void setRainbow() {
		candle.clearAllAnimations();
		candle.setControl(new RainbowAnimation(8, 8 + stripLength));
	}

	public void setRgbFadeAnimation() {
		candle.clearAllAnimations();
		candle.setControl(new RgbFadeAnimation(8, 8 + stripLength));
	}

	public void setTwinkle() {
		candle.clearAllAnimations();
		candle.setControl(new TwinkleAnimation(8, 8 + stripLength));
	}

	public void setStrobe() {
		candle.clearAllAnimations();
		candle.setControl(new StrobeAnimation(8, 8 + stripLength).withFrameRate(FLASH_RATE).withColor(new RGBWColor(color)));
	}

	public void setFastStrobe() {
		candle.clearAllAnimations();
		candle.setControl(new StrobeAnimation(8, 8 + stripLength).withFrameRate(FLASH_RATE * 4).withColor(new RGBWColor(color)));
	}

	public void setStatic() {
		candle.clearAllAnimations();
		candle.setControl(new SolidColor(8, 8 + stripLength).withColor(new RGBWColor(color)));
	}

	public void lightsOff() {
		candle.clearAllAnimations();
		candle.setControl(new SolidColor(8 , 8 + stripLength).withColor(new RGBWColor(0, 0, 0, 0)));
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
