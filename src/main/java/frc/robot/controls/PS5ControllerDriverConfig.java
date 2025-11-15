package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.drive_comm.DriveToPose;
import frc.robot.commands.gpm.SetArm;
import frc.robot.commands.gpm.SetElevator;
import frc.robot.commands.gpm.intake.IntakeCoral;
import frc.robot.commands.gpm.outtake.OuttakeGamePiece;
import frc.robot.commands.gpm.outtake.SetOuttakeGamePiece;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.Constants.OuttakeLocation;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.controllers.PS5Controller;
import lib.controllers.PS5Controller.DPad;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller.PS5Button;

/**
 * Driver controls for the PS5 controller
 */
public class PS5ControllerDriverConfig extends BaseDriverConfig {
    private final BooleanSupplier slowModeSupplier = ()->false;
    private final PS5Controller driver = new PS5Controller(Constants.DRIVER_JOY);

    private ArmComp arm;
    private Elevator elevator;
    private Intake intake;
    private Indexer indexer;
    private Outtake outtake;
    private Pose2d alignmentPose = null;
    private int selectedDirection = 0;

    public PS5ControllerDriverConfig(Drivetrain drive, ArmComp arm, Elevator elevator, Intake intake, Indexer indexer, Outtake outtake) {
        super(drive);
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake; 
    }

    public void configureControls() {
        Trigger menu = driver.get(DPad.UP);
        // Reset the yaw. Mainly useful for testing/driver practice
        driver.get(PS5Button.CREATE).onTrue(new InstantCommand(() -> getDrivetrain().setYaw(
            new Rotation2d(Robot.getAlliance() == Alliance.Blue ? 0 : Math.PI)
        )));

        // Cancel commands
        driver.get(PS5Button.RB).and(menu.negate()).onTrue(new InstantCommand(()->{
            if(elevator != null){
                if(outtake != null && outtake.coralLoaded()){
                    elevator.setSetpoint(ElevatorConstants.INTAKE_STOW_SETPOINT);
                }else{
                    elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                }
            }
            if(outtake != null){
                outtake.setMotor(0);
            }
            if(intake != null){
                intake.stow();
                intake.stopRollers();
            }
            if(indexer != null){
                indexer.stop();
            }
            if(arm != null){
                if(outtake != null && outtake.coralLoaded()){
                    arm.setSetpoint(ArmConstants.STOW_SETPOINT);
                }else{
                    arm.setSetpoint(ArmConstants.START_ANGLE);
                }
            }
            getDrivetrain().setIsAlign(false);
            getDrivetrain().setDesiredPose(()->null);
            alignmentPose = null;
            selectedDirection = 0;
            CommandScheduler.getInstance().cancelAll();
        }));

        // Align wheels
        driver.get(PS5Button.MUTE).onTrue(new FunctionalCommand(
            ()->getDrivetrain().setStateDeadband(false),
            getDrivetrain()::alignWheels,
            interrupted->getDrivetrain().setStateDeadband(true),
            ()->false, getDrivetrain()).withTimeout(2));

        // outtake algae to barge (net)
        driver.get(PS5Button.TOUCHPAD).onTrue(
            new SequentialCommandGroup(
                new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.NET),
                new OuttakeGamePiece(elevator, arm, outtake, OuttakeLocation.NET)
            )
        );

        // outtake algae to processor
        driver.get(DPad.DOWN).and(menu.negate()).onTrue(
            new SequentialCommandGroup(
                new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.PROCESSOR),
                new OuttakeGamePiece(elevator, arm, outtake, OuttakeLocation.PROCESSOR)
            )
        );

        // Alignment for the reef 
        driver.get(DPad.LEFT).toggleOnTrue(new InstantCommand(() -> {
            selectedDirection = -1;
        }));

        driver.get(DPad.RIGHT).toggleOnTrue(new InstantCommand(() -> {
            selectedDirection = 1;
        }));

        // TODO: bottom algae position intake (take off the reef)
    

        // TODO: top algae position intake (take off the reef)

        // I DONT KNOW IF THIS WORKS 
        // intake coral 
        // THIS WORKs when we use the instant command () -> intake.unstow() but it doesn't work when we call the command
        driver.get(PS5Button.RIGHT_TRIGGER).toggleOnTrue(
            new ConditionalCommand(
                new IntakeCoral(intake, indexer, elevator, outtake, arm), 
                // new InstantCommand(() -> intake.unstow()),
                new DoNothing(), 
                // if the coral is not loaded, intake (above)
                () -> !outtake.coralLoaded())
        );
     
       // outtake coral
        driver.get(PS5Button.RIGHT_TRIGGER).onTrue(
            new ConditionalCommand(
                new OuttakeGamePiece(elevator, arm, outtake), 
                new DoNothing(), 
                // if the coral is loaded, outtake (above)
                () -> outtake.coralLoaded())
        );

        // set coral L1
        driver.get(PS5Button.CROSS).and(menu.negate()).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setAlignmentPose(false, true)),
                new ConditionalCommand(
                    // with vision
                    new ParallelCommandGroup(
                        new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L1),
                        new DriveToPose(getDrivetrain(), ()->alignmentPose)
                    ),
                    // Manual
                    new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L1),  
                    () -> selectedDirection != 0),
                new ConditionalCommand(
                    new OuttakeGamePiece(elevator, arm, outtake), 
                    new DoNothing(), 
                    () -> selectedDirection != 0
                )
            )
        );

        // outtake coral L2 and outtake algae L2
        driver.get(PS5Button.SQUARE).and(menu.negate()).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setAlignmentPose(false, false)),
                new ConditionalCommand(
                    // with vision
                    new ParallelCommandGroup(
                        new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L2),
                        new DriveToPose(getDrivetrain(), ()->alignmentPose)
                    ),
                    // Manual
                    new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L2),  
                    () -> selectedDirection != 0),
                new ConditionalCommand(
                    new OuttakeGamePiece(elevator, arm, outtake), 
                    new DoNothing(), 
                    () -> selectedDirection != 0
                )
            )
        );

        // outtake coral L3 and outtake algae L3
        driver.get(PS5Button.CIRCLE).and(menu.negate()).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setAlignmentPose(false, false)),
                new ConditionalCommand(
                    // with vision
                    new ParallelCommandGroup(
                        new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L3),
                        new DriveToPose(getDrivetrain(), ()->alignmentPose)
                    ),
                    // Manual
                    new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L3),  
                    () -> selectedDirection != 0),
                new ConditionalCommand(
                    new OuttakeGamePiece(elevator, arm, outtake), 
                    new DoNothing(), 
                    () -> selectedDirection != 0
                )
            )
        );

        // outtake coral L4
        driver.get(PS5Button.TRIANGLE).and(menu.negate()).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> setAlignmentPose(true, false)),
                new ConditionalCommand(
                    // with vision
                    new ParallelCommandGroup(
                        new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L4),
                        new DriveToPose(getDrivetrain(), ()->alignmentPose)
                    ),
                    // Manual
                    new SetOuttakeGamePiece(elevator, arm, OuttakeLocation.L4),  
                    () -> selectedDirection != 0),
                new ConditionalCommand(
                    new OuttakeGamePiece(elevator, arm, outtake), 
                    new DoNothing(), 
                    () -> selectedDirection != 0
                )
            )
        );

        // climb?
       

        




    }

     /**
     * Sets the drivetrain's alignmetn pose to the nearest reef branch or algae location
     * @param isAlgae True for algae, false for branches
     * @param isTrue for left branch, false for right, ignored for algae
     * @param l4 If the robot should align to the L4 scoring pose
     */
    private void setAlignmentPose(boolean isAlgae, boolean isLeft, boolean l4, boolean l1){
        Translation2d drivePose = getDrivetrain().getPose().getTranslation();
        int closestId = 0;
        double closestDist = 20;
        boolean isRed = Robot.getAlliance() == Alliance.Red;
        int start = isRed ? 5 : 16;
        for(int i = 0; i < 6; i++){
            double dist = FieldConstants.APRIL_TAGS.get(start+i).pose.toPose2d().getTranslation().getDistance(drivePose);
            if(dist < closestDist){
                closestDist = dist;
                closestId = start+i+1;
            }
        }
        if(isAlgae){
            alignmentPose = VisionConstants.REEF.fromAprilTagIdAlgae(closestId).pose;
        }else{
            if(l4){
                alignmentPose = VisionConstants.REEF.fromAprilTagIdAndPose(closestId, isLeft).l4Pose;
            }else if(l1){
                alignmentPose = VisionConstants.REEF.fromAprilTagIdAndPose(closestId, isLeft).l1Pose;
            }else{
                alignmentPose = VisionConstants.REEF.fromAprilTagIdAndPose(closestId, isLeft).pose;
            }
        }
    }

    /**
     * Sets the drivetrain's alignmetn pose to the nearest reef branch with the selected direction
     * @param l4 If the robot should align to the L4 scoring pose
     * @param l1 If the robot should align to the L1 scoring pose
     */
    private void setAlignmentPose(boolean l4, boolean l1){
        if(selectedDirection == 0){
            alignmentPose = null;
            return;
        }
        setAlignmentPose(false, selectedDirection < 0, l4, l1);
    }
    
    /**
     * Sets the drivetrain's alignmetn pose to the nearest reef branch with the selected direction
     * @param l4 If the robot should align to the L4 scoring pose
     */
    private void setAlignmentPose(boolean l4){
        setAlignmentPose(l4, false);
    }
    
    @Override
    public double getRawSideTranslation() {
        return driver.get(PS5Axis.LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return driver.get(PS5Axis.LEFT_Y);
    }

    @Override
    public double getRawRotation() {
        return driver.get(PS5Axis.RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(driver.get(PS5Axis.RIGHT_X), -driver.get(PS5Axis.RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return Math.hypot(driver.get(PS5Axis.RIGHT_X), driver.get(PS5Axis.RIGHT_Y));
    }

    @Override
    public boolean getIsSlowMode() {
        return slowModeSupplier.getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return false;
    }

    public void startRumble(){
        driver.rumbleOn();
    }

    public void endRumble(){
        driver.rumbleOff();
    }

    
}
