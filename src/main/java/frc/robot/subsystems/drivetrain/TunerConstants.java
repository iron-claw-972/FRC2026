package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IdConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;

/**
 * should be equivalent to the Phoenix Tuner-generated constants from the wizard (that i didnt use ;p)
 *
 */
public final class TunerConstants {
    private TunerConstants() {}

    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100.0)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(2.49)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(DriveConstants.P_VALUES[0])
            .withKI(DriveConstants.I_VALUES[0])
            .withKD(DriveConstants.D_VALUES[0])
            .withKS(DriveConstants.S_VALUES[0])
            .withKV(DriveConstants.V_VALUES[0])
            .withKA(DriveConstants.A_VALUES[0]);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT))
                    .withStatorCurrentLimitEnable(DriveConstants.DRIVE_ENABLE_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(Amps.of(DriveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT))
                    .withSupplyCurrentLimitEnable(DriveConstants.DRIVE_ENABLE_CURRENT_LIMIT)
                    .withSupplyCurrentLowerLimit(Amps.of(DriveConstants.DRIVE_PEAK_CURRENT_LIMIT))
                    .withSupplyCurrentLowerTime(Seconds.of(DriveConstants.DRIVE_PEAK_CURRENT_DURATION)));

    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT))
                    .withStatorCurrentLimitEnable(DriveConstants.STEER_ENABLE_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(Amps.of(DriveConstants.STEER_CONTINUOUS_CURRENT_LIMIT))
                    .withSupplyCurrentLimitEnable(DriveConstants.STEER_ENABLE_CURRENT_LIMIT)
                    .withSupplyCurrentLowerLimit(Amps.of(DriveConstants.STEER_PEAK_CURRENT_LIMIT))
                    .withSupplyCurrentLowerTime(Seconds.of(DriveConstants.STEER_PEAK_CURRENT_DURATION)));

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    private static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    public static final CANBus kCANBus = DriveConstants.DRIVE_MOTOR_CAN;

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<
                            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(DriveConstants.DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(DriveConstants.STEER_GEAR_RATIO)
                    .withCouplingGearRatio(4.5) //TODO add this, we didn't actually compensate for this in our prev code
                    .withWheelRadius(DriveConstants.WHEEL_RADIUS)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withSlipCurrent(Amps.of(50))
                    .withSpeedAt12Volts(MetersPerSecond.of(DriveConstants.MAX_SPEED))
                    .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                    .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withDriveMotorInitialConfigs(driveInitialConfigs)
                    .withSteerMotorInitialConfigs(steerInitialConfigs)
                    .withEncoderInitialConfigs(encoderInitialConfigs)
                    .withSteerInertia(KilogramSquareMeters.of(0.01))
                    .withDriveInertia(KilogramSquareMeters.of(0.01))
                    .withSteerFrictionVoltage(Volts.of(0.2))
                    .withDriveFrictionVoltage(Volts.of(0.2));

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(IdConstants.PIGEON)
            .withPigeon2Configs(pigeonConfigs);

    private static Angle offset(double degrees) {
        return edu.wpi.first.units.Units.Rotations.of(degrees / 360.0);
    }

    private static boolean steerInverted() {
        return DriveConstants.INVERT_STEER_MOTOR
                == com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
    }

    private static boolean driveInverted() {
        return DriveConstants.INVERT_DRIVE_MOTOR
                == com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
    }

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            module(ModuleConstants module, double x, double y, boolean rightSide) {
        return ConstantCreator.createModuleConstants(
                module.getSteerPort(),
                module.getDrivePort(),
                module.getEncoderPort(),
                offset(module.getSteerOffset()),
                Meters.of(x),
                Meters.of(y),
                driveInverted() ^ rightSide,
                steerInverted(),
                DriveConstants.MODULE_CONSTANTS.canCoderInvert);
    }

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
    FrontLeft = module(
            ModuleConstants.FRONT_LEFT,
            DriveConstants.MODULE_LOCATIONS[0].getX(),
            DriveConstants.MODULE_LOCATIONS[0].getY(),
            false);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
    FrontRight = module(
            ModuleConstants.FRONT_RIGHT,
            DriveConstants.MODULE_LOCATIONS[1].getX(),
            DriveConstants.MODULE_LOCATIONS[1].getY(),
            true);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
    BackLeft = module(
            ModuleConstants.BACK_LEFT,
            DriveConstants.MODULE_LOCATIONS[2].getX(),
            DriveConstants.MODULE_LOCATIONS[2].getY(),
            false);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
    BackRight = module(
            ModuleConstants.BACK_RIGHT,
            DriveConstants.MODULE_LOCATIONS[3].getX(),
            DriveConstants.MODULE_LOCATIONS[3].getY(),
            true);

    public static GeneratedDrivetrain createDrivetrain() {
        return new GeneratedDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    public static class TunerSwerveDrivetrain
            extends com.ctre.phoenix6.swerve.SwerveDrivetrain<
                    com.ctre.phoenix6.hardware.TalonFX,
                    com.ctre.phoenix6.hardware.TalonFX,
                    com.ctre.phoenix6.hardware.CANcoder> {
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    com.ctre.phoenix6.hardware.TalonFX::new,
                    com.ctre.phoenix6.hardware.TalonFX::new,
                    com.ctre.phoenix6.hardware.CANcoder::new,
                    drivetrainConstants,
                    modules);
        }

        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    com.ctre.phoenix6.hardware.TalonFX::new,
                    com.ctre.phoenix6.hardware.TalonFX::new,
                    com.ctre.phoenix6.hardware.CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    modules);
        }
    }
}
