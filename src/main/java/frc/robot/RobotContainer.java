package frc.robot;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.DefaultDriveCommand;
import frc.robot.commands.gpm.AutoShootCommand;
import frc.robot.commands.gpm.ClimbCommand;
import frc.robot.commands.vision.ShutdownAllPis;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.Climb.LinearClimb;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.hood.Hood;
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

  private Command auto = new DoNothing();

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  private Operator operator = null;
  private LinearClimb linearClimb = null;
  // TODO: move to correct robot and put the correct port?
  private PS5Controller ps5 = new PS5Controller(0);



  // Auto Command selection
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {
    // display the current robot id on smartdashboard
    SmartDashboard.putString("RobotID", robotId.toString());

    // Filling the SendableChooser on SmartDashboard
    autoChooserInit();

    // dispatch on the robot
    switch (robotId) {
      case TestBed1:
        break;

      case TestBed2:
        break;

      default:

      case PrimeJr: // AKA Valence
        spindexer = new Spindexer();
        intake = new Intake();

      case WaffleHouse: // AKA Betabot
        // turret = new Turret();
        shooter = new Shooter();
        hood = new Hood();

      case SwerveCompetition: // AKA "Vantage"

      case BetaBot: // AKA "Pancake"
        //vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);
        // fall-through

      case Vivace:
        //linearClimb = new LinearClimb();

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

        initializeAutoBuilder();
        registerCommands();
        PathGroupLoader.loadPathGroups();
        // Load the auto command
        try {
          PathPlannerAuto.getPathGroupFromAutoFile("Command Name");
          auto = new PathPlannerAuto("Path Name");
        } catch (IOException | ParseException e) {
          e.printStackTrace();
        }
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        break;
    }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // TODO: verify this claim.
    // LiveWindow is causing periodic loop overruns
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);

    SmartDashboard.putData("Shutdown Orange Pis", new ShutdownAllPis());
    autoChooserInit();
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
          Logger.recordOutput("Auto/ChassisSpeeds", chassisSpeeds);
          drive.setChassisSpeeds(chassisSpeeds, false); // problem??
        },
        AutoConstants.AUTO_CONTROLLER,
        AutoConstants.CONFIG,
        getAllianceColorBooleanSupplier(),
        drive);
  }

  public void registerCommands() {
    if (intake != null){
      NamedCommands.registerCommand("Extend intake", new InstantCommand(()-> intake.extend()));
      NamedCommands.registerCommand("Retract intake", new InstantCommand(()-> intake.retract()));
    }

    if (intake != null && spindexer != null){ 
      NamedCommands.registerCommand("Intake", new ParallelCommandGroup(
        new InstantCommand(()->intake.spin(IntakeConstants.SPEED)),
        new InstantCommand(()-> spindexer.setSpindexer(SpindexerConstants.spindexerMaxPower))
      ));
    }

    if (turret != null && drive != null && hood != null && shooter != null && spindexer != null){
      NamedCommands.registerCommand("Auto shoot", new AutoShootCommand(turret, drive, hood, shooter, spindexer));
    }

    if (linearClimb != null && drive != null){
      NamedCommands.registerCommand("Climb", new ClimbCommand(linearClimb, drive, ps5));
    }

  }

  /**
   * Initialize the SendableChooser on the SmartDashboard.
   * Fill the SendableChooser with available Commands.
   */
  public void autoChooserInit() {
    // add the options to the Chooser
    autoChooser.setDefaultOption("Do nothing", new DoNothing());
    autoChooser.addOption("Do nada", new DoNothing());
    autoChooser.addOption("Spin my wheels", new DoNothing());
    autoChooser.addOption("Hello world", new InstantCommand(() -> System.out.println("Hello world")));

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
    // get the selected Command
    Command autoSelected = autoChooser.getSelected();

    return autoSelected;
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
}
