package frc.robot.commands.gpm.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gpm.SetArm;
import frc.robot.commands.gpm.SetElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Constants.ReefLevel;
import frc.robot.subsystems.Arm.ArmComp;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Outtake.Outtake;
import frc.robot.subsystems.intake.Intake;

public class OuttakeCoral extends Command {
    private Elevator elevator;
    private ArmComp arm;
    private Intake intake;
    private Outtake outtake;
    private double speed;
    private Timer timer;

    private enum Phase {
        Outtaking,
        InitialLowering,
        FinalLowering,
        Done
    }

    private Phase phase; 

    public OuttakeCoral(Elevator elevator, ArmComp arm, Intake intake, Outtake outtake, ReefLevel reefLevel){
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.outtake = outtake;
        speed = reefLevel.speed;
        timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.restart();
        outtake.outtake(); 
        phase = Phase.Outtaking;
    }

    @Override
    public void execute(){
        switch (phase){
            case Outtaking:
                if (timer.hasElapsed(0.5)){
                    outtake.setMotor(0);
                    arm.setSetpoint(ArmConstants.INTAKE_SETPOINT);
                    elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                    phase = Phase.InitialLowering;
                }
                break;
                
            case InitialLowering:
                if (elevator.getPosition() < ElevatorConstants.L2_SETPOINT) {
                    // if the arm is at fully stowed then continue lowering elevator 
                    if (arm.atSetpoint()) {
                        elevator.setSetpoint(ElevatorConstants.STOW_SETPOINT);
                        phase = Phase.FinalLowering;
                    // if arm is not fully stowed, stop elevator and wait until fully stowed then lower elevator
                    } else {
                        // Stop a little bit under L2 to make 
                        elevator.setSetpoint(ElevatorConstants.L2_SETPOINT - .1); 
                    }
                } 
                break;
            case FinalLowering:
                if (elevator.atSetpoint()) {
                    phase = Phase.Done; 
                }
                break;
            case Done: 
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if (phase == Phase.Done) {
            return true; 
        } 
        return false; 
    }



}
