package frc.robot.util;


import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubActive {
  static public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }


    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }


    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };
    // 67 is a clear indication of an error
    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }


  static public Optional<Double> timeToActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return Optional.empty();
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return Optional.of(0.0);
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return Optional.empty();
    }


    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return Optional.of(0.0);
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return Optional.of(0.0);
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return Optional.of(0.0);
    } else if (matchTime > 105) {
      // Shift 1
      if (shift1Active) {
        return Optional.of(matchTime - 105.0);
      } else {
        return Optional.empty();
      }
    } else if (matchTime > 80) {
      // Shift 2


      if (!shift1Active) {
        return Optional.of(matchTime - 80.0);
      } else {
        return Optional.empty();
      }
    } else if (matchTime > 55) {
      // Shift 3


      if (shift1Active) {
        return Optional.of(matchTime - 55.0);
      } else {
        return Optional.empty();
      }
    } else if (matchTime > 30) {
      // Shift 4


      if (!shift1Active) {
        return Optional.of(matchTime - 30.0);
      } else {
        return Optional.empty();
      }
    } else {
      // End game, hub always active.
      return Optional.of(0.0);
    }
  }

  static public Optional<Double> timeToInactive() {

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
      alliance = Alliance.Red;
    } else {
      alliance = Alliance.Blue;
    }

    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return Optional.of(0.0);
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return Optional.empty();
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return Optional.of(0.0);
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return Optional.of(0.0);
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return Optional.of(0.0);
    } else if (matchTime > 105) {
      // Shift 1
      if (shift1Active) {
        return Optional.of(matchTime - 105.0);
      } else {
        return Optional.empty();
      }
    } else if (matchTime > 80) {
      // Shift 2

      if (!shift1Active) {
        return Optional.of(matchTime - 80.0);
      } else {
        return Optional.empty();
      }
    } else if (matchTime > 55) {
      // Shift 3

      if (shift1Active) {
        return Optional.of(matchTime - 55.0);
      } else {
        return Optional.empty();
      }
    } else if (matchTime > 30) {
      // Shift 4

      if (!shift1Active) {
        return Optional.of(matchTime - 30.0);
      } else {
        return Optional.empty();
      }
    } else {
      // End game, hub always active.
      return Optional.of(0.0);
    }
  }

  static public boolean wonAuto() {
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return false;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return false;
      }
    }


    var alliance = DriverStation.getAlliance().get();
    boolean x;
    if (alliance == Alliance.Red) {
      x = redInactiveFirst;
    } else {
      x = !redInactiveFirst;
    }
    return x;

  }
}
