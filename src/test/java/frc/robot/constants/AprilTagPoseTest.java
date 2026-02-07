package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.Vision.Vision;

/**
 * Tests if all of the AprilTags are in the right spot
 */
public class AprilTagPoseTest {
  /**
   * Tests if there are the right number of AprilTags, that the tags in Vision match the ones in FieldConstants, and that they are on the right side of the field
   */
  @Test
  public void testTagPoses() {
    // Construct the vision instance
    //   makes the field layout
    Vision vision = new Vision(new ArrayList<Pair<String, Function<Double, Transform3d>>>());

    // we should have 32 tags
    assertEquals(32, FieldConstants.field.getTags().size());
    assertEquals(32, vision.getAprilTagFieldLayout().getTags().size());

    // Check each tag in the field layout
    for (int i = 0; i < vision.getAprilTagFieldLayout().getTags().size(); i++) {
      // The expected tagId. The ArrayList is zero-based and our tags start at 1, so the tagId is i+1.
      int tagId = i + 1;

      // Get the poses from the two sources
      // From the ArrayList<AprilTag> source
      AprilTag apriltag1 = FieldConstants.field.getTags().get(i);
      Pose3d p1 = apriltag1.pose;
      // From the vision source
      Pose3d p2 = vision.getTagPose(tagId);

      // Check the tag id in the ArrayList
      assertEquals(tagId, apriltag1.ID);

      // Make sure the points match
      assertEquals(p1.getX(), p2.getX(), 0.0001);
      assertEquals(p1.getY(), p2.getY(), 0.0001);
      assertEquals(p1.getZ(), p2.getZ(), 0.0001);

      // Make sure the rotations match
      assertEquals(p1.getRotation().getX(), p2.getRotation().getX(), 0.0001);
      assertEquals(p1.getRotation().getY(), p2.getRotation().getY(), 0.0001);
      assertEquals(p1.getRotation().getZ(), p2.getRotation().getZ(), 0.0001);

      // 1-16 should be on the right, and 17-36 should be on the left
      if(tagId > 16){
        assertTrue(p1.getX() < FieldConstants.field.getFieldLength()/2);
      }else{
        assertTrue(p1.getX() > FieldConstants.field.getFieldLength()/2);
      }
    }
  }

  @Test
  public void testReefTags(){
    List<Pose3d> redPoses = FieldConstants.field.getTags().subList(1, 16).stream().map(tag->tag.pose).toList();
    List<Pose3d> bluePoses = FieldConstants.field.getTags().subList(17, 32).stream().map(tag->tag.pose).toList();
    Pose3d redCenter = findCenter(redPoses);
    Pose3d blueCenter = findCenter(bluePoses);
    
    // The tags should be symmetrical, so the total rotation should be 0
    assertEquals(redCenter.getRotation().getX(), 0, 0.0001);
    assertEquals(blueCenter.getRotation().getX(), 0, 0.0001);
    assertEquals(redCenter.getRotation().getY(), 0, 0.0001);
    assertEquals(blueCenter.getRotation().getY(), 0, 0.0001);
    assertEquals(MathUtil.angleModulus(redCenter.getRotation().getZ()), Math.PI, 0.0001);
    assertEquals(MathUtil.angleModulus(blueCenter.getRotation().getZ()), 0, 0.0001);

    // Y are symmetrical diagonally
    assertEquals(redCenter.getY(), FieldConstants.field.getFieldWidth() - blueCenter.getY(), 0.01);
    // Z should be equal
    assertEquals(redCenter.getZ(), blueCenter.getZ(), 0.0001);

    // X should be mirrored
    assertEquals(redCenter.getX(), FieldConstants.field.getFieldLength()-blueCenter.getX(), 0.01);

    // Compare each matching pair of tags
    for(int i = 1; i < 17; i++){
      Pose3d red = FieldConstants.field.getTagPose(i).get();
      Pose3d blue = FieldConstants.field.getTagPose(i+16).get();
      assertEquals(red.getY(), FieldConstants.field.getFieldWidth() - blue.getY(), 0.01);
      assertEquals(red.getZ(), blue.getZ(), 0.0001);
      assertEquals(red.getX(), FieldConstants.field.getFieldLength()-blue.getX(), 0.01);
      assertEquals(MathUtil.angleModulus(red.getRotation().getZ()), MathUtil.angleModulus(blue.getRotation().getZ() + Math.PI), 0.0001);
    }
  }

  /**
   * Gets the center pose with the sum of the rotations, used for checking the reef
   * @param poses The poses to find the center of
   * @return A pose with the translation at the center and the sum of the rotations
   */
  private Pose3d findCenter(List<Pose3d> poses){
    double x = 0;
    double y = 0;
    double z = 0;
    Rotation3d rot = new Rotation3d();
    for(Pose3d pose : poses){
      x += pose.getX();
      y += pose.getY();
      z += pose.getZ();
      rot = rot.plus(pose.getRotation());
    }
    x /= poses.size();
    y /= poses.size();
    z /= poses.size();
    return new Pose3d(x, y, z, rot);
  }
}
