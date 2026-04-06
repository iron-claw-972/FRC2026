package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CurrentModes;
import frc.robot.commands.LogCommand;
import frc.robot.commands.drive_comm.DefaultDriveCommand;
import frc.robot.commands.gpm.AutoShootCommand;
import frc.robot.commands.gpm.BrownOutControl;
import frc.robot.commands.gpm.ClimbDriveCommand;
import frc.robot.commands.gpm.IntakeMovementCommand;
import frc.robot.commands.gpm.LockedShoot;
import frc.robot.commands.gpm.RunSpindexer;
import frc.robot.commands.gpm.Superstructure;
import frc.robot.commands.vision.ShutdownAllPis;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.Climb.LinearClimb;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision.DetectedObject;
import frc.robot.util.Vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private Drivetrain drive = null;
  private Vision vision = null;
  private Turret turret = null;
  private Shooter shooter = null;
  private Hood hood = null;
  private Spindexer spindexer = null;
  private Intake intake = null;

  // this is inside addAuto()
  // private Command auto = new DoNothing();

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  private Operator operator = null;
  private LinearClimb linearClimb = null;
  private LED led = null;

  // TODO: move to correct robot and put the correct port?
  private PS5Controller ps5 = new PS5Controller(0);

  // auto Command selection
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // custom match timer that counts down each phase
  private final Timer matchTimer = new Timer();
  private double currentPhaseDuration = 20.0; // default auto duration
  private boolean timerActive = false;
  private String currentPhase = "none";
  
  // phase durations
  private static final double AUTO_DURATION = 20.0;
  private static final double TRANSITION_SHIFT_DURATION = 10.0;
  private static final double SHIFT_DURATION = 25.0;
  private static final double ENDGAME_DURATION = 30.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {
    // display the current robot id on smartdashboard
    if (!Constants.DISABLE_SMART_DASHBOARD) {
      SmartDashboard.putString("RobotID", robotId.toString());

      SmartDashboard.putNumber("Match Time", 0.0);
    }

    // Filling the SendableChooser on SmartDashboard

    // dispatch on the robot
    switch (robotId) {
      case TestBed1:
        break;

      case TestBed2:
        // led = new LED();
        // led.setDefaultCommand(new LEDDefaultCommand(led));
        break;

      default:

      case PrimeJr: // AKA Valence
        spindexer = new Spindexer();
        intake = new Intake();
        led = new LED();

      case WaffleHouse: // AKA Betabot
        turret = new Turret();
        shooter = new Shooter();
        hood = new Hood();
      
      case TwinBot:

      case SwerveCompetition: // AKA "Vantage"

      case BetaBot: // AKA "Pancake"
        vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);
        // fall-through

      case Vivace:

      case Phil: // AKA "IHOP"

      case Vertigo: // AKA "French Toast"
        drive = new Drivetrain(vision, new GyroIOPigeon2());
        driver = new PS5ControllerDriverConfig(drive, shooter, turret, hood, intake, spindexer, linearClimb);
        operator = new Operator(drive);

        // Detected objects need access to the drivetrain
        DetectedObject.setDrive(drive);

        // SignalLogger.start();
        driver.configureControls();
        operator.configureControls();

        registerCommands();
        PathGroupLoader.loadPathGroups();

        initializeAutoBuilder();
        autoChooserInit();

        // put the Chooser on the SmartDashboard
        SmartDashboard.putData("Auto chooser", autoChooser);

        SmartDashboard.putData("Lock Shooting", new LockedShoot(turret, drive, hood, shooter));

        if (turret != null) {
          turret.setDefaultCommand(new Superstructure(turret, drive, hood, shooter, spindexer));
        }

        if (shooter != null && spindexer != null && turret != null && intake != null && hood != null && drive != null) {
          CommandScheduler.getInstance().schedule(new BrownOutControl(shooter, spindexer, turret, intake, hood, drive)); //TODO pick one or the other
          CommandScheduler.getInstance().schedule(new CurrentModes(shooter, spindexer, turret, intake, hood, drive));
        }
        
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        break;
    }

	if (intake != null && hood != null && turret != null)
		// CommandScheduler.getInstance().schedule(new HardstopWarning(hood, intake, turret)); (no more crt for this)
    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    CommandScheduler.getInstance().schedule(new LogCommand());

    // TODO: verify this claim.
    // LiveWindow is causing periodic loop overruns
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);

    if (!Constants.DISABLE_SMART_DASHBOARD) {
      SmartDashboard.putData("Shutdown Orange Pis", new ShutdownAllPis());
    }
  }

  /**
   * Sets whether the drivetrain uses vision toupdate odometry
   */
  public void setVisionEnabled(boolean enabled) {
    if (drive != null)
      drive.setVisionEnabled(enabled);
  }

  public void initializeAutoBuilder() {
    AutoBuilder.configure(
        () -> drive.getPose(),
        (pose) -> {
          drive.resetOdometry(pose);
        },
        () -> drive.getChassisSpeeds(),
        (chassisSpeeds) -> {
          if (!Constants.DISABLE_LOGGING) {
            Logger.recordOutput("Auto/ChassisSpeeds", chassisSpeeds);
          }
          drive.setChassisSpeeds(chassisSpeeds, false); // problem??
        },
        AutoConstants.AUTO_CONTROLLER,
        AutoConstants.CONFIG,
        getAllianceColorBooleanSupplier(),
        drive);
  }

  private boolean seizing;

  public void registerCommands() {

    if (intake != null) {

      NamedCommands.registerCommand("Extend Intake", new InstantCommand(() -> {
        intake.extend();
      }));
      NamedCommands.registerCommand("Retract Intake", new InstantCommand(() -> intake.retract()));
      NamedCommands.registerCommand("Intermediate Extend", new InstantCommand(() -> intake.intermediateExtend()));
      NamedCommands.registerCommand("Spin Intake Rollers", new InstantCommand(() -> intake.spinStart()));
      NamedCommands.registerCommand("Stop Intake Rollers", new InstantCommand(() -> intake.spinStop()));

      NamedCommands.registerCommand("Start Intake Seizure", new InstantCommand(() -> {
        seizing = true;
        CommandScheduler.getInstance().schedule(new IntakeMovementCommand(intake).until(() -> !seizing));
      }));
      NamedCommands.registerCommand("Stop Intake Seizure", new InstantCommand(() -> {
        seizing = false;
      }));
    }

    if (turret != null && drive != null && hood != null && shooter != null && spindexer != null && intake != null) {
      Command runSpindexer = new RunSpindexer(spindexer, turret, hood, intake);
      NamedCommands.registerCommand("Auto shoot", new AutoShootCommand(turret, drive, hood, shooter, spindexer));
      NamedCommands.registerCommand("Start Spindexer",
          new InstantCommand(() -> CommandScheduler.getInstance().schedule(runSpindexer)));
      NamedCommands.registerCommand("Stop Spindexer", new InstantCommand(() -> runSpindexer.cancel()));
      NamedCommands.registerCommand("Reset Spindexer", new InstantCommand(() -> spindexer.resetSpindexer()));
      NamedCommands.registerCommand("Reset Reset Angle", new InstantCommand(() -> spindexer.resetResetAngle()));
    }

    if (hood != null) {

      NamedCommands.registerCommand("Hood Down", new InstantCommand(() -> {
        hood.forceHoodDown(true);
      }));
      NamedCommands.registerCommand("Stop Hood Down", new InstantCommand(() -> {
        hood.forceHoodDown(false);
      }));
    }

    if (linearClimb != null && drive != null) {
      NamedCommands.registerCommand("Climb", new ClimbDriveCommand(linearClimb, drive));
    }

  }

  public void addAuto(String name) {
    try {
      Command auto = new PathPlannerAuto(name);
      autoChooser.addOption(name, auto);
    }
    // is this the right one??
    catch (AutoBuilderException e) {
      e.printStackTrace();
      System.out.println("HELLOOOO AUTO \"" + name + "\" NOT FOUND");
    }
  }

  /**
   * Initialize the SendableChooser on the SmartDashboard.
   * Fill the SendableChooser with available Commands.
   */
  public void autoChooserInit() {
    // add the options to the Chooser
    String defaultAuto = "Trial Auto Path";
    String leftSideAuto = "Left Week V1";
    String rightSideAuto = "Right Week V1";
    String shootOnlyAuto = "Shoot Only Left Week V1";
    String leftLiberalSwipe = "LeftLiberalDoubleSwipe";
    String rightLiberalSwipe = "RightLiberalDoubleSwipe";
    String leftLiberalSwipeTranslation = "LeftLiberalDoubleSwipeTranslation";
    String leftConservativeSwipe = "LeftConservativeDoubleSwipe";

    autoChooser.setDefaultOption("Default", new PathPlannerAuto(defaultAuto));
    addAuto(leftSideAuto);
    addAuto(rightSideAuto);
    addAuto(shootOnlyAuto);
    addAuto(leftConservativeSwipe);
    addAuto(leftLiberalSwipe);
    addAuto(rightLiberalSwipe);
    addAuto(leftLiberalSwipeTranslation);


    // put the Chooser on the SmartDashboard
    SmartDashboard.putData("Auto chooser", autoChooser);
  }

  public static BooleanSupplier getAllianceColorBooleanSupplier() {
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red
      // alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
  }

  public boolean brownout() {
    if (RobotController.getBatteryVoltage() < 6.0) {
      return true;
    } else {
      return false;
    }
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }

  public void logComponents() {
    if (!Constants.LOG_MECHANISMS)
      return;

    Logger.recordOutput(
        "ComponentPoses",
        new Pose3d[] {
        // Subsystem Pose3ds
        });
  }

  /** Updates SmartDashboard values that need to be refreshed every loop */
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();
    String newPhase;
    
    if (matchTime > 130) {
      newPhase = "auto";
    } else if (matchTime > 120) {
      newPhase = "transition_shift";
    } else if (matchTime > 95) {
      newPhase = "shift1";
    } else if (matchTime > 70) {
      newPhase = "shift2";
    } else if (matchTime > 45) {
      newPhase = "shift3";
    } else if (matchTime > 20) {
      newPhase = "shift4";
    } else if (matchTime > 0) {
      newPhase = "endgame";
    } else {
      newPhase = "disabled";
    }
    
    if (!newPhase.equals(currentPhase)) {
      currentPhase = newPhase;
      matchTimer.reset();
      matchTimer.start();
      timerActive = true;
      
      switch (currentPhase) {
        case "auto":
          currentPhaseDuration = AUTO_DURATION;
          break;
        case "transition_shift":
          currentPhaseDuration = TRANSITION_SHIFT_DURATION;
          break;
        case "shift1":
        case "shift2":
        case "shift3":
        case "shift4":
          currentPhaseDuration = SHIFT_DURATION;
          break;
        case "endgame":
          currentPhaseDuration = ENDGAME_DURATION;
          break;
        default:
          currentPhaseDuration = 0.0;
          timerActive = false;
      }
    }
    
    double countdownTime = 0.0;
    if (timerActive && currentPhaseDuration > 0) {
      double elapsed = matchTimer.get();
      countdownTime = Math.max(0, currentPhaseDuration - elapsed);
    }
    if (!Constants.DISABLE_SMART_DASHBOARD) {
      SmartDashboard.putNumber("Phase Countdown", countdownTime);
      SmartDashboard.putString("Current Phase", currentPhase);
    }
    
    if (matchTime > 0) {
      if (!Constants.DISABLE_SMART_DASHBOARD) {
        SmartDashboard.putNumber("Match Time", matchTime);
      }
    }
    if (!Constants.DISABLE_SMART_DASHBOARD) {
      SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
    }
  }
}
