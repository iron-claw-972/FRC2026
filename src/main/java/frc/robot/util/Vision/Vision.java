package frc.robot.util.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.util.MathUtils;

// Vision and it's commands are adapted from Iron Claw's FRC2023
public class Vision {
	// The field layout. Instance variable
	private AprilTagFieldLayout aprilTagFieldLayout;
	// A list of the cameras on the robot.
	private ArrayList<VisionCamera> cameras = new ArrayList<>();

	private ArrayList<PhotonCamera> objectCameras;

	// these should always be the same length
	private ArrayList<DetectedObject> detectedObjects;
	private ArrayList<VisionObjInfo> objectInfo;

	private VisionSystemSim visionSim;

	private boolean sawTag = false;

	// Array of tags to use, null or empty array to use all tags
	private int[] onlyUse = null;

	/**
	 * Creates a new instance of Vision and sets up the cameras and field layout
	 */
	public Vision(ArrayList<Pair<String, Transform3d>> camList) {
		// Load field layout
		aprilTagFieldLayout = new AprilTagFieldLayout(FieldConstants.APRIL_TAGS, FieldConstants.FIELD_LENGTH,
				FieldConstants.FIELD_WIDTH);

		// Sets the origin to the right side of the blue alliance wall
		aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

		if (VisionConstants.ENABLED) {
			// Puts the cameras in an array list
			for (int i = 0; i < camList.size(); i++) {
				cameras.add(new VisionCamera(camList.get(i).getFirst(), camList.get(i).getSecond()));
			}

			if (RobotBase.isSimulation()) {
				visionSim = new VisionSystemSim("Vision");
				visionSim.addAprilTags(aprilTagFieldLayout);
				for (VisionCamera c : cameras) {
					PhotonCameraSim cameraSim = new PhotonCameraSim(c.camera);
					cameraSim.enableDrawWireframe(true);
					cameraSim.prop.setAvgLatencyMs(30);
					cameraSim.prop.setCalibration(1280, 720, Rotation2d.fromDegrees(78));
					visionSim.addCamera(cameraSim, c.photonPoseEstimator.getRobotToCameraTransform());
				}
			}
		}

		if (VisionConstants.OBJECT_DETECTION_ENABLED) {
			for (String i : VisionConstants.OBJECT_DETECTION_CAMERAS.keySet()) {
				objectCameras.add(new PhotonCamera(i));
			}
		}
	}

	/**
	 * Update the list of detected objects, which can then be retrieved with
	 * getDetectedObjects.
	 *
	 * Should be called once and only once per tick.
	 */
	public void updateDetectedObjects() {
		if (!VisionConstants.OBJECT_DETECTION_ENABLED)
			return;

		this.detectedObjects = new ArrayList<DetectedObject>();
		this.objectInfo = new ArrayList<VisionObjInfo>();

		// note: this may include results from previous frames and may be empty
		for (PhotonCamera camera : this.objectCameras) {
			var results = camera.getAllUnreadResults(); // can be empty
			for (PhotonPipelineResult result : results) {
				boolean hasTargets = result.hasTargets();
				if (!hasTargets)
					continue;

				var targets = result.getTargets();
				for (PhotonTrackedTarget target : targets) {
					Transform3d transform = VisionConstants.OBJECT_DETECTION_CAMERAS.get(camera.getName());
					int classId = target.getDetectedObjectClassID();
					double yaw = target.getYaw();
					double pitch = target.getPitch();
					this.detectedObjects.add(new DetectedObject(yaw, pitch, classId, transform));
					this.objectInfo.add(new VisionObjInfo(yaw, pitch));
				}
			}
		}
	}

	/**
	 * Stores all of the detected objects in an array
	 * 
	 * @return The array of DetectedObjects
	 */
	public DetectedObject[] getDetectedObjects() {
		if (!VisionConstants.OBJECT_DETECTION_ENABLED)
			return new DetectedObject[0];

		var sampleArray = new DetectedObject[] {};
		return this.detectedObjects.toArray(sampleArray);
	}

	/**
	 * Get extra vision info not stored in the DetectedObject class.
	 * 
	 * Unlike that class, this is specific to the robot's position when the image was taken.
	 */
	public VisionObjInfo[] getObjectInfo() {
		var sampleArray = new VisionObjInfo[]{};
		return this.objectInfo.toArray(sampleArray);
	}

	/**
	 * Get all the yaw values for detected objects as an array
	 */
	public double[] getObjectYaws() {
		var arr = new double[this.objectInfo.size()];
		for (int i = 0; i < this.objectInfo.size(); i++) {
			arr[i] = this.objectInfo.get(i).yaw;
		}
		return arr;
	}

	/**
	 * Get all the pitch values for detected objects as an array
	 */
	public double[] getObjectPitches() {
		var arr = new double[this.objectInfo.size()];
		for (int i = 0; i < this.objectInfo.size(); i++) {
			arr[i] = this.objectInfo.get(i).pitch;
		}
		return arr;
	}

	/**
	 * Returns the closest game piece in front of the robot
	 * 
	 * @param maxAngle           The maximum angle between the angle to the object
	 *                           and the robot's heading or rotation to use, in
	 *                           radians
	 * @param relativeToVelocity Whether to compare the angle to the robot's heading
	 *                           or rotation, true for heading
	 * @return The best DetectedObject
	 */
	public DetectedObject getBestGamePiece(double maxAngle, boolean relativeToVelocity) {
		DetectedObject[] objects = getDetectedObjects();
		DetectedObject best = null;
		double closest = Double.POSITIVE_INFINITY;
		for (DetectedObject object : objects) {
			double dist = object.getDistance();
			if (object.isGamePiece()
					&& Math.abs(relativeToVelocity ? object.getVelocityRelativeAngle() : object.getAngle()) < maxAngle
					&& dist < closest) {
				closest = dist;
				best = object;
			}
		}
		return best;
	}

	/**
	 * Gets the pose as a Pose2d using PhotonVision
	 * 
	 * @param referencePoses The reference poses in order of preference, null poses
	 *                       will be skipped
	 * @return The pose of the robot, or null if it can't see april tags
	 */
	public Pose2d getPose2d(Pose2d... referencePoses) {
		Pose2d referencePose = new Pose2d();
		for (Pose2d checkReferencePose : referencePoses) {
			if (checkReferencePose != null) {
				referencePose = checkReferencePose;
				break;
			}
		}
		ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(referencePose);

		if (estimatedPoses.size() == 0)
			return null;

		if (estimatedPoses.size() == 1)
			return estimatedPoses.get(0).estimatedPose.toPose2d();

		if (estimatedPoses.size() == 2) {
			return new Pose2d(
					estimatedPoses.get(0).estimatedPose.toPose2d().getTranslation()
							.plus(estimatedPoses.get(1).estimatedPose.toPose2d().getTranslation())
							.div(2),

					new Rotation2d(MathUtils.modulusMidpoint(
							estimatedPoses.get(0).estimatedPose.toPose2d().getRotation().getRadians(),
							estimatedPoses.get(1).estimatedPose.toPose2d().getRotation().getRadians(),
							-Math.PI, Math.PI)));
		}

		// The average translation is just the average of all of the translations (sum
		// divided by total)
		// Average angle is similar, except every step needs to use a modulus, since -π
		// is the same angle as π
		// This calculation is essentially newAverage = (oldAverage * valuesInOldAverage
		// + nextValue) / newNumberOfValues
		Translation2d translation = new Translation2d();
		double angle = 0;
		for (int i = 0; i < estimatedPoses.size(); i++) {
			translation = translation.plus(estimatedPoses.get(i).estimatedPose.toPose2d().getTranslation());
			angle = MathUtils.modulusInterpolate(angle,
					estimatedPoses.get(i).estimatedPose.toPose2d().getRotation().getRadians(), 1.0 / (i + 1), -Math.PI,
					Math.PI);
		}

		return new Pose2d(translation.div(estimatedPoses.size()), new Rotation2d(angle));
	}

	public AprilTagFieldLayout getAprilTagFieldLayout() {
		return aprilTagFieldLayout;
	}

	/**
	 * Gets the pose of an april tag
	 * 
	 * @param id AprilTag id (1-8)
	 * @return Pose3d of the AprilTag
	 */
	public Pose3d getTagPose(int id) {
		if (id < 1 || id > getAprilTagFieldLayout().getTags().size()) {
			System.out.println("Tried to find the pose of april tag " + id);
			return null;
		}
		return getAprilTagFieldLayout().getTags().get(id - 1).pose;
	}

	/**
	 * Returns where it thinks the robot is
	 * 
	 * @param referencePose The pose to use as a reference, usually the previous
	 *                      robot pose
	 * @param yawFunction   A unary operator that takes a timestamp and returns the
	 *                      yaw at that time
	 * @return An array list of estimated poses, one for each camera that can see an
	 *         april tag
	 */
	public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d referencePose) {
		return getEstimatedPoses(referencePose, ignoree -> referencePose.getRotation().getRadians());
	}

	/**
	 * Returns where it thinks the robot is
	 * 
	 * @param referencePose The pose to use as a reference, usually the previous
	 *                      robot pose
	 * @param yawFunction   A unary operator that takes a timestamp and returns the
	 *                      yaw at that time
	 * @return An array list of estimated poses, one for each camera that can see an
	 *         april tag
	 */
	public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d referencePose, DoubleUnaryOperator yawFunction) {
		ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
		for (int i = 0; i < cameras.size(); i++) {
			if (VisionConstants.USE_MANUAL_CALCULATIONS) {
				for (EstimatedRobotPose pose : cameras.get(i).getEstimatedPose(yawFunction)) {
					if (pose != null) {
						estimatedPoses.add(pose);
					}
				}
			} else {
				for (EstimatedRobotPose pose : cameras.get(i).getEstimatedPose(referencePose)) {
					// If the camera can see an april tag that exists, add it to the array list
					// April tags that don't exist might return a result that is present but doesn't
					// have a pose
					if (pose.estimatedPose != null) {
						estimatedPoses.add(pose);

					}
				}
			}
		}
		if (estimatedPoses.size() > 1) {
			Translation2d average = new Translation2d();
			for (EstimatedRobotPose pose : estimatedPoses) {
				average = average.plus(pose.estimatedPose.getTranslation().toTranslation2d());
			}
			average = average.div(estimatedPoses.size());
			for (int i = estimatedPoses.size() - 1; i >= 0; i--) {
				if (estimatedPoses.get(i).estimatedPose.getTranslation().toTranslation2d()
						.getDistance(average) > VisionConstants.MAX_POSE_DIFFERENCE / 2) {
					estimatedPoses.remove(i);
				}
			}
		}
		return estimatedPoses;
	}

	/**
	 * Updates the robot's odometry with vision
	 * 
	 * @param poseEstimator The pose estimator to update
	 * @param yawFunction   A function that returns the yaw as a double given the
	 *                      timestamp
	 * @param slipped       True if the wheels have slipped, false otherwise
	 */
	public void updateOdometry(SwerveDrivePoseEstimator poseEstimator, DoubleUnaryOperator yawFunction,
			boolean slipped) {
		// Simulate vision
		// 2 ifs to avoid warning
		if (VisionConstants.ENABLED_SIM) {
			if (RobotBase.isSimulation()) {
				visionSim.update(poseEstimator.getEstimatedPosition());
			}
		}

		this.updateDetectedObjects(); // TODO: put this somewhere else

		sawTag = false;

		// An array list of poses returned by different cameras
		ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(poseEstimator.getEstimatedPosition(),
				yawFunction);
		for (EstimatedRobotPose estimatedPose : estimatedPoses) {
			// Continue if this pose doesn't exist
			if (estimatedPose.timestampSeconds < 0 || !onField(estimatedPose.estimatedPose.toPose2d())
					|| Timer.getFPGATimestamp() < estimatedPose.timestampSeconds
					|| Timer.getFPGATimestamp() > estimatedPose.timestampSeconds + 1) {
				continue;
			}

			poseEstimator.addVisionMeasurement(
					estimatedPose.estimatedPose.toPose2d(),
					estimatedPose.timestampSeconds,
					slipped ? VisionConstants.VISION_STD_DEVS_2 : VisionConstants.VISION_STD_DEVS);
			sawTag = true;
		}
	}

	/**
	 * Updates each camera's inputs for logging
	 */
	public void updateInputs() {
		for (VisionCamera c : cameras) {
			c.updateInputs();
		}
	}

	/**
	 * If vision saw any April tags last frame
	 * 
	 * @return If vision saw an April tag last frame
	 */
	public boolean canSeeTag() {
		return sawTag;
	}

	/**
	 * Enable or disable a single camera
	 * 
	 * @param index   The camera index
	 * @param enabled If it should be enabled or disabled
	 */
	public void enableCamera(int index, boolean enabled) {
		try {
			cameras.get(index).enable(enabled);
		} catch (IndexOutOfBoundsException e) {
			DriverStation.reportWarning("Camera index " + index + " is out of bounds", false);
		}
	}

	/**
	 * Sets the cameras to only use April tag in the specified array
	 * 
	 * @param ids The ids of the tags to use, null or empty array to use all
	 */
	public void onlyUse(int[] ids) {
		onlyUse = ids;
	}

	/**
	 * Checks if a pose is on the field
	 * 
	 * @param pose The pose to check
	 * @return If the pose is on the field
	 */
	public static boolean onField(Pose2d pose) {
		return pose != null && pose.getX() > 0 && pose.getX() < FieldConstants.FIELD_LENGTH && pose.getY() > 0
				&& pose.getY() < FieldConstants.FIELD_WIDTH;
	}

	/**
	 * Checks if a pose is on or near the field
	 * 
	 * @param pose The pose to check
	 * @return If the pose is within an area with twice the length and width of the
	 *         field
	 */
	public static boolean nearField(Pose2d pose) {
		return pose != null && pose.getX() > -FieldConstants.FIELD_LENGTH / 2
				&& pose.getX() < FieldConstants.FIELD_LENGTH * 1.5 && pose.getY() > -FieldConstants.FIELD_WIDTH / 2
				&& pose.getY() < FieldConstants.FIELD_WIDTH * 1.5;
	}

	private class VisionCamera implements VisionIO {
		private PhotonCamera camera;
		private PhotonPoseEstimator photonPoseEstimator;
		private Pose2d lastPose;
		private double lastTimestamp = 0;
		private boolean enabled = true;
		private final VisionIOInputs inputs = new VisionIOInputs();

		/**
		 * Stores information about a camera
		 * 
		 * @param cameraName The name of the camera on PhotonVision
		 * @param robotToCam The transformation from the robot to the camera
		 */
		public VisionCamera(String cameraName, Transform3d robotToCam) {
			camera = new PhotonCamera(cameraName);
			photonPoseEstimator = new PhotonPoseEstimator(
					aprilTagFieldLayout,
					VisionConstants.POSE_STRATEGY,
					robotToCam);
			photonPoseEstimator.setMultiTagFallbackStrategy(VisionConstants.MULTITAG_FALLBACK_STRATEGY);
			photonPoseEstimator.setReferencePose(new Pose2d());
			lastPose = null;
		}

		/**
		 * Gets the estimated poses from the camera
		 * 
		 * @param referencePose Pose to use for reference, usually the previous
		 *                      estimated robot pose
		 * @return estimated robot poses
		 */
		public ArrayList<EstimatedRobotPose> getEstimatedPose(Pose2d referencePose) {
			photonPoseEstimator.setReferencePose(referencePose);

			ArrayList<EstimatedRobotPose> list = new ArrayList<>();

			if (!enabled) {
				return list;
			}

			for (PhotonPipelineResult cameraResult : inputs.results) {
				if (!cameraResult.hasTargets() || cameraResult.getTimestampSeconds() < 0) {
					continue;
				}

				// if there is a target detected and the timestamp exists,
				// check the ambiguity isn't too high
				List<PhotonTrackedTarget> targetsUsed = cameraResult.targets;
				for (int i = targetsUsed.size() - 1; i >= 0; i--) {
					// Remove it from the list if it should not be used or if it has too high of an
					// ambiguity
					if (!useTag(targetsUsed.get(i).getFiducialId())
							|| targetsUsed.get(i).getPoseAmbiguity() > VisionConstants.HIGHEST_AMBIGUITY
							|| targetsUsed.get(i).bestCameraToTarget.getTranslation()
									.getNorm() > VisionConstants.MAX_DISTANCE) {
						targetsUsed.remove(i);
					}
				}

				// If there are no targets, the timestamp doesn't exist, or there there is only
				// 1 tag and the constant is set to only use 2 tags, continue
				if (targetsUsed.size() == 0 || cameraResult.getTimestampSeconds() < 0
						|| targetsUsed.size() == 1 && VisionConstants.ONLY_USE_2_TAGS) {
					continue;
				}

				// Set strategy to single tag if there is only 1 good tag and update
				photonPoseEstimator.setPrimaryStrategy(targetsUsed.size() > 1 ? VisionConstants.POSE_STRATEGY
						: VisionConstants.MULTITAG_FALLBACK_STRATEGY);
				Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cameraResult);

				if (pose.isPresent() && pose.get() != null && onField(pose.get().estimatedPose.toPose2d())) {
					double timestamp = cameraResult.getTimestampSeconds();

					// If the pose moved too much, don't use it
					if (lastPose == null
							|| lastPose.getTranslation().getDistance(pose.get().estimatedPose.toPose2d()
									.getTranslation()) > DriveConstants.MAX_SPEED * 1.25 * (timestamp - lastTimestamp)
							|| timestamp < lastTimestamp) {
						lastPose = pose.get().estimatedPose.toPose2d();
						lastTimestamp = timestamp;
						continue;
					}

					// Otherwise, add the pose to the list
					lastPose = pose.get().estimatedPose.toPose2d();
					lastTimestamp = timestamp;
					list.add(pose.get());
				}
			}
			return list;
		}

		/**
		 * Updates the VisionIOInputs object with the results from PhotonVision for
		 * logging
		 */
		@Override
		public void updateInputs() {
			inputs.connected = camera.isConnected();
			inputs.results = camera.getAllUnreadResults();

			Logger.processInputs("Vision/" + camera.getName(), inputs);

			// Mechanical Advantage's vision logging
			// // Read new camera observations
			// Set<Short> tagIds = new HashSet<>();
			// List<PoseObservation> poseObservations = new LinkedList<>();
			// for (var result : camera.getAllUnreadResults()) {
			// // Update latest target observation
			// if (result.hasTargets()) {
			// inputs.latestTargetObservation =
			// new TargetObservation(
			// Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
			// Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
			// } else {
			// inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new
			// Rotation2d());
			// }
			// }
			// // Add pose observation
			// if (result.multitagResult.isPresent()) { // Multitag result
			// var multitagResult = result.multitagResult.get();

			// // Calculate robot pose
			// Transform3d fieldToCamera = multitagResult.estimatedPose.best;
			// Transform3d fieldToRobot =
			// fieldToCamera.plus(VisionConstants.APRIL_TAG_CAMERAS.get(0).getSecond().inverse());
			// Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(),
			// fieldToRobot.getRotation());

			// // Calculate average tag distance
			// double totalTagDistance = 0.0;
			// for (var target : result.targets) {
			// totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
			// }

			// // Add tag IDs
			// tagIds.addAll(multitagResult.fiducialIDsUsed);

			// // Add observation
			// poseObservations.add(
			// new PoseObservation(
			// result.getTimestampSeconds(), // Timestamp
			// robotPose, // 3D pose estimate
			// multitagResult.estimatedPose.ambiguity, // Ambiguity
			// multitagResult.fiducialIDsUsed.size(), // Tag count
			// totalTagDistance / result.targets.size(), // Average tag distance
			// PoseObservationType.PHOTONVISION)); // Observation type

			// } else if (!result.targets.isEmpty()) { // Single tag result
			// var target = result.targets.get(0);

			// // Calculate robot pose
			// var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
			// if (tagPose.isPresent()) {
			// Transform3d fieldToTarget =
			// new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
			// Transform3d cameraToTarget = target.bestCameraToTarget;
			// Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
			// Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
			// Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(),
			// fieldToRobot.getRotation());

			// // Add tag ID
			// tagIds.add((short) target.fiducialId);

			// // Add observation
			// poseObservations.add(
			// new PoseObservation(
			// result.getTimestampSeconds(), // Timestamp
			// robotPose, // 3D pose estimate
			// target.poseAmbiguity, // Ambiguity
			// 1, // Tag count
			// cameraToTarget.getTranslation().getNorm(), // Average tag distance
			// PoseObservationType.PHOTONVISION)); // Observation type
			// }
			// }
			// }
		}

		/**
		 * Gets the pose using manual calculations
		 * 
		 * @param yawFunction A unary operator that takes a timestamp and returns the
		 *                    yaw at that time
		 * @return A list of estimated poses as EstimatedRobotPoses
		 */
		public ArrayList<EstimatedRobotPose> getEstimatedPose(DoubleUnaryOperator yawFunction) {
			ArrayList<EstimatedRobotPose> list = new ArrayList<>();

			// Do nothing if this camera is disabled
			if (!enabled) {
				return list;
			}

			// The latest camera results
			for (PhotonPipelineResult result : inputs.results) {
				// TODO: This could be improved by averaging all targets instead of only using 1

				// Continue if the target doesn't exist or it should be ignored
				if (!result.hasTargets())
					continue;
				// Gets the best target to use for the calculations
				PhotonTrackedTarget target = result.getBestTarget();
				// I don't know why this would happen, but keep it in just in case
				if (target == null) {
					continue;
				}
				// Continue if the id is too high or too low
				int id = target.getFiducialId();
				if (!useTag(id) || target.bestCameraToTarget.getTranslation().getNorm() > VisionConstants.MAX_DISTANCE
						|| target.poseAmbiguity > VisionConstants.HIGHEST_AMBIGUITY) {
					continue;
				}
				// Stores target pose and robot to camera transformation for easy access later
				Pose3d targetPose = FieldConstants.APRIL_TAGS.get(id - 1).pose;
				Transform3d robotToCamera = photonPoseEstimator.getRobotToCameraTransform();

				double timestamp = result.getTimestampSeconds();
				double yaw = yawFunction.applyAsDouble(timestamp);

				// Get the tag position relative to the robot, assuming the robot is on the
				// ground
				Translation3d translation = target.getBestCameraToTarget().getTranslation()
						.rotateBy(robotToCamera.getRotation());
				translation = translation// .times((targetPose.getZ()-robotToCamera.getZ())/translation.getZ())
						.plus(robotToCamera.getTranslation())
						.rotateBy(new Rotation3d(0, 0, yaw))

						// Invert it to get the robot position relative to the April tag
						// Multiply by a constant. I don't know why this works, but it was consistently
						// 10% off in 2023 Fall Semester
						.times(-VisionConstants.DISTANCE_SCALE)
						// Get the field relative robot pose
						.plus(targetPose.getTranslation());
				try {
					// Adds an EstimatedRobotPose
					list.add(new EstimatedRobotPose(
							new Pose3d(translation.getX(), translation.getY(), 0, new Rotation3d(0, 0, yaw)),
							timestamp,
							List.of(target),
							VisionConstants.POSE_STRATEGY));
				} catch (Exception e) {
					DriverStation.reportError("Error creating EstimatedRobotPose", true);
				}
			}
			return list;
		}

		public boolean useTag(int id) {
			// Never use tags that don't exist
			if (id <= 0 || id > FieldConstants.APRIL_TAGS.size()) {
				return false;
			}
			// Return false if it is in the list of tags to ignore
			for (int id2 : VisionConstants.TAGS_TO_IGNORE) {
				if (id == id2) {
					return false;
				}
			}
			// If it's in the array to only use and not in the array to ignore, return true
			for (int j = 0; onlyUse != null && j < onlyUse.length; j++) {
				if (id == onlyUse[j]) {
					return true;
				}
			}
			// If it isn't in the array to only use, only reutrn true if the array is
			// empty/null
			return onlyUse == null || onlyUse.length == 0;
		}

		/**
		 * Enables or disables this camera
		 * 
		 * @param enable If it should be enabled or disabled
		 */
		public void enable(boolean enable) {
			enabled = enable;
		}
	}

	public record VisionObjInfo(double yaw, double pitch) { }
}
