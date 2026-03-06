package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.FieldZone;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class PhaseManager {
    
    public enum WantedState {
        IDLE,
        SHOOTING,
        PASSING
    }

    public enum CurrentState {
        IDLE,
        STARTING_UP,
        TURNING_AROUND,
        SHOOTING,
        PASSING
    }

    private WantedState wantedState = WantedState.SHOOTING;
    private CurrentState currentState = CurrentState.IDLE;

    public void update(Pose2d drivePose, Shooter shooter, Turret turret) {
        updateWantedState(drivePose);
        updateCurrentState(drivePose, shooter, turret);
    }

    private void updateCurrentState(Pose2d drivePose, Shooter shooter, Turret turret) {
        // if shooter is not trying to run -- idle
        if (shooter.getTargetVelocity() == 0.0) {
            currentState = CurrentState.IDLE;
            return;
        }
        // if shooter velocity not ready yet -- starting up
        if (!shooter.atTargetSpeed()) { // TODO: but then what happens when the ball goes in and the velocity dips??
            currentState = CurrentState.STARTING_UP;
            return;
        }
        // if turret is not at setpoint -- turning around
        if (!turret.atSetpoint()) {
            currentState = CurrentState.TURNING_AROUND;
            return;
        }
        
        FieldZone zone = FieldConstants.getZone(drivePose.getTranslation());
        if (zone == FieldConstants.FieldZone.ALLIANCE) {
            currentState = CurrentState.SHOOTING;
        } else {
            currentState = CurrentState.PASSING;
        }
    }

    private void updateWantedState(Pose2d drivePose) {
        FieldZone zone = FieldConstants.getZone(drivePose.getTranslation());
        if (zone == FieldConstants.FieldZone.ALLIANCE) {
            wantedState = WantedState.SHOOTING;
        } else {
            wantedState = WantedState.PASSING;
        }
    }

    public WantedState getWantedState() { 
        return wantedState; 
    }
    public CurrentState getCurrentState() { 
        return currentState; 
    }
    
    public boolean isIdle() { 
        return wantedState == WantedState.IDLE; 
    }
    
    public boolean shouldFeed() {
        // TODO: I'm gonna comment out starting up until i find a solution
        return !(currentState == CurrentState.TURNING_AROUND); //&& !(currentState == CurrentState.STARTING_UP);
    }

    public Translation2d getTarget() {
        return wantedState == WantedState.SHOOTING ? 
            FieldConstants.getHubTranslation().toTranslation2d() 
            : FieldConstants.getOppositionTranslation(true).toTranslation2d();
    }
}