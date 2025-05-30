package frc.robot.commands.vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.MathUtils;
import frc.robot.util.Vision.Vision;

/**
 * Calculates standard deviations for vision
 */
public class CalculateStdDevs extends Command {
  private final Vision vision;
  private ArrayList<Pose2d> poses;
  private int arrayLength;
  private Timer endTimer;
  private Drivetrain drive;

  /**
   * Constructor for CalculateStdDevs
   * @param posesToUse the amount of poses to take the standard deviation of. More poses will take more time.
   * @param vision The vision
   */
  public CalculateStdDevs(int posesToUse, Vision vision, Drivetrain drive) {
    this.vision = vision;
    this.drive = drive;
    arrayLength = posesToUse;
    endTimer = new Timer();
  }

  /**
   * Resets the pose array
   */
  @Override
  public void initialize() {
    // create the ArrayList of poses to store
    // an ArrayList prevents issues if the command ends early, and makes checking if the command has finished easy
    poses = new ArrayList<Pose2d>();

    drive.setVisionEnabled(false);
  }

  /**
   * Adds a pose to the array
   */
  @Override
  public void execute() {
    Pose2d pose = vision.getPose2d(drive.getPose());
    // If the pose exists, add it to the first open spot in the array
    if (pose != null) {
      // if we see a pose, reset the timer (it will be started the next time it doesn't get a pose)
      endTimer.stop();
      endTimer.reset();
      // add the pose to our data
      poses.add(pose);
      if(poses.size()%10==0){
        System.out.printf("%.0f%% done\n", ((double)poses.size())/arrayLength * 100);
      }
    } else {
      endTimer.start();
      // If kStdDevCommandEndTime seconds have passed since it saw an April tag, stop the command
      // Prevents it from running forever
      if (endTimer.hasElapsed(10)) {
        cancel();
      }
    }
  }

  /**
   * Calculates the standard deviation
   */
  @Override
  public void end(boolean interrupted) {
    drive.setVisionEnabled(true);

    // If the array is empty, don't try to calculate std devs
    if (poses.size() == 0) {
      System.out.println("There are no poses in the array\nTry again where the robot can see an April tag.");
      return;
    }
    
    // create arrays of the poses by X, Y, and Rotation for calculations
    double[] xArray = new double[poses.size()];
    double[] yArray = new double[poses.size()];
    double[] rotArray = new double[poses.size()];

    // copy the values into the arrays
    for (int i = 0; i < poses.size(); i++) {
      xArray[i] = poses.get(i).getX();
      yArray[i] = poses.get(i).getY();
      rotArray[i] = poses.get(i).getRotation().getRadians();
    }

    // Calculate the standard deviations
    double stdDevX = MathUtils.stdDev(xArray);
    double stdDevY = MathUtils.stdDev(yArray);
    double stdDevRot = MathUtils.stdDev(rotArray);
    
    // Find distance to tag
    double distance;
    try{
      distance = vision.getEstimatedPoses(drive.getPose()).get(0).targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm();
    }catch(Exception e){
        System.out.println("Could not see a target");
        distance = -1;
    }
    
    // Print and log values
    System.out.printf("Standard deviation values:\nX: %.5f\nY: %.5f\nRotation: %.5f\nDistance: %.5f\n",
      stdDevX, stdDevY, stdDevRot, distance);
  }

  /**
   * Returns if the command is finished
   * @return If the array is full
   */
  @Override
  public boolean isFinished() {
    return poses.size() >= arrayLength;
  }
}