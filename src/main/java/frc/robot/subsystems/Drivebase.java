// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SwerveTeleop;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Targeting.FieldTarget;
import frc.robot.utils.SmartPIDController;

public class Drivebase extends SubsystemBase {

  private static Drivebase drivebase;
  AHRS gyro = new AHRS(I2C.Port.kOnboard);

  // Shuffleboard/Glass visualizations of robot position on the field.
  private final Field2d integratedOdometryPrint = new Field2d();
  private final Field2d visualOdometryPrint = new Field2d();

  PhotonCamera camera = new PhotonCamera(Constants.PhotonVision.PHOTON_CAMERA_NAME);
  ChassisSpeeds speeds;

  // Position used for targeting.
  Optional<Translation2d> fieldTarget;

  AprilTagFieldLayout aprilTagFieldLayout;
  double maxVelocity = 0;
  SmartPIDController headingController = new SmartPIDController(
      Constants.PIDControllers.HeadingControlPID.KP, Constants.PIDControllers.HeadingControlPID.KI,
      Constants.PIDControllers.HeadingControlPID.KD, "Heading Controller",
      Constants.PIDControllers.HeadingControlPID.SMART_PID_ACTIVE);

  // locations of the modules, x positive forward y positive left
  Translation2d leftFrontLocation =
      new Translation2d(Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
          Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d rightFrontLocation =
      new Translation2d(Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
          Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d leftBackLocation =
      new Translation2d(Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
          Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y));

  Translation2d rightBackLocation =
      new Translation2d(Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
          Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y));

  SwerveModule frontLeft =
      new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.FRONT_LEFT_MODULE);

  SwerveModule frontRight =
      new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.FRONT_RIGHT_MODULE);

  SwerveModule backLeft =
      new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.BACK_LEFT_MODULE);

  SwerveModule backRight =
      new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.BACK_RIGHT_MODULE);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontLocation,
      rightFrontLocation, leftBackLocation, rightBackLocation);

  SwerveDrivePoseEstimator odometry =
      new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getGyroAngle()),
          new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
              backLeft.getPosition(), backRight.getPosition()},
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

  PhotonPoseEstimator visualOdometry;

  private Drivebase() {
    // gyro.reset();

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Exception reading AprilTag Field JSON " + e.toString());
    }

    visualOdometry = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.PhotonVision.ROBOT_TO_CAMERA);

    // The robot should have the same heading as the heading specified here on
    // startup.
    resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    SmartDashboard.putData("Integrated Odometry", integratedOdometryPrint);
    SmartDashboard.putData("Visual Odometry", visualOdometryPrint);

    configurePathPlanner();
    headingController.enableContinuousInput(0, 360);

    SmartDashboard.putNumber("testTurnP", 0);
    SmartDashboard.putNumber("testTurnI", 0);
    SmartDashboard.putNumber("testTurnD", 0);

    // Setting the targetingPoint to Optional.empty() (there is no target until
    // button is pressed).
    fieldTarget = Optional.empty();
  }

  /** run in teleop init to set swerve as default teleop command */
  public void setSwerveAsDefaultCommand() {
    setDefaultCommand(new SwerveTeleop(drivebase, OI.getInstance()));
  }

  /**
   * Used to get the angle reported by the gyro. This method is private, and should only be called
   * when creating/updating the SwervePoseEstimator. Otherwise, call getRobotHeading instead.
   */
  private double getGyroAngle() {
    double angle = gyro.getAngle();
    SmartDashboard.putNumber("gyro", -angle);

    // Negative because gyro reads differently than wpilib.
    return -angle;
  }

  public double getGyroRoll () {
    double roll = gyro.getRoll();
    SmartDashboard.putNumber("gyro roll", roll);

    return roll;
  }
  /**
   * Call this method instead of getGyroAngle(). This method returns the robot's heading according
   * to the integrated odometry. This allows for an accurate heading measurement, even if the gyro
   * is inaccurate.
   * 
   * @return The heading of the robot according to the integrated odometry, in degrees. Note:
   *         Measurement is 0-360 degrees instead of continuous.
   */
  public double getRobotHeading() {
    return getRobotPose().getRotation().getDegrees();
  }

  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond,
      boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond), Units.degreesToRadians(degreesPerSecond),
          Rotation2d.fromDegrees(getRobotHeading()));
    } else {
      speeds = new ChassisSpeeds(Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond), Units.degreesToRadians(degreesPerSecond));
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Caps the module speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED);
    setModuleStates(moduleStates);
  }

  // Used for keeping robot heading in the right direction using PID and the
  // targeting buttion
  public void setDriveTurnPos(double xFeetPerSecond, double yFeetPerSecond, boolean fieldRelative) {
    double degreesPerSecond;
    degreesPerSecond = headingController.calculate(getRobotHeading());
    setDrive(xFeetPerSecond, yFeetPerSecond, degreesPerSecond, fieldRelative);
  }

  public void setHeadingController(double setpoint) {
    headingController.setSetpoint(setpoint);
    SmartDashboard.putNumber("Heading Setpoint", setpoint);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    backLeft.setState(states[2]);
    backRight.setState(states[3]);
  }

  /** Returns the estimated position of the odometry */
  public Pose2d getRobotPose() {
    return odometry.getEstimatedPosition();
  }

  /** Reset the position of the odometry */
  public void resetOdometry(Pose2d resetPose) {
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()},
        resetPose);
  }

  /** Update odometry position. Call this function every loop in periodic. */
  public void updateOdometry() {

    // Update the mechanical odometry
    odometry.update(Rotation2d.fromDegrees(getGyroAngle()),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()});

    Optional<EstimatedRobotPose> updatedVisualPose = visualOdometry.update();
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    Transform3d distanceToTargetTransform;
    SmartDashboard.putBoolean("Targets found", hasTargets);

    // Check if there are targets
    if (updatedVisualPose.isPresent() && hasTargets) {

      // try/catch statement to ensure getBestCameraToTarget() won't crash code
      try {
        distanceToTargetTransform = result.getBestTarget().getBestCameraToTarget();
      } catch (NullPointerException e) {
        return;
      }

      // Calculate the uncertainty of the vision measurement based on distance from
      // the best
      // AprilTag target.
      EstimatedRobotPose pose = updatedVisualPose.get();
      double distanceToTarget = Math.sqrt(Math.pow(distanceToTargetTransform.getX(), 2)
          + Math.pow(distanceToTargetTransform.getY(), 2));
      SmartDashboard.putNumber("Distance to target", distanceToTarget);
      Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(new SimpleMatrix(
          new double[] {distanceToTarget * Constants.PhotonVision.DISTANCE_UNCERTAINTY_PROPORTIONAL,
              distanceToTarget * Constants.PhotonVision.DISTANCE_UNCERTAINTY_PROPORTIONAL,
              distanceToTarget * Constants.PhotonVision.ROTATIONAL_UNCERTAINTY_PROPORTIONAL}));

      // Add vision measurement/update FieldLayout prints
      visualOdometryPrint.setRobotPose(pose.estimatedPose.toPose2d());
      odometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
          uncertainty);
    }
    integratedOdometryPrint.setRobotPose(getRobotPose());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public static Drivebase getInstance() {
    if (drivebase == null) {
      drivebase = new Drivebase();
    }
    return drivebase;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(frontLeft.getSwerveState(),
        frontRight.getSwerveState(), backLeft.getSwerveState(), backRight.getSwerveState());

    return chassisSpeeds;
  }

  public void setDriveChassisSpeed(ChassisSpeeds chassisSpeeds) {
    setDrive(Units.metersToFeet(chassisSpeeds.vxMetersPerSecond),
        Units.metersToFeet(chassisSpeeds.vyMetersPerSecond),
        Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond),
        // path planner uses robot reletive drive command.
        false);
  }

  /**
   * Sets the current target point used for targeting.
   * 
   * @param target The target to point at (FieldTarget enum value)
   */
  public void setFieldTarget(FieldTarget fieldTarget) {

    Optional<Translation2d> fieldTargetOptional;

    if (fieldTarget == null) {
      fieldTargetOptional = Optional.empty();
    } else {
      fieldTargetOptional = Optional
          .of(new Translation2d(fieldTarget.get().get().getX(), fieldTarget.get().get().getY()));
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();

    // Relying on short circuting here to check if optional value is Alliance.Red.
    if (fieldTargetOptional.isPresent() && alliance.isPresent() && alliance.get() == Alliance.Red) {
      this.fieldTarget = Optional.of(new Translation2d(
          Constants.FIELD_X_LENGTH / 2
              + (Constants.FIELD_X_LENGTH / 2 - fieldTargetOptional.get().getX()),
          fieldTargetOptional.get().getY()));
    } else {
      this.fieldTarget = fieldTargetOptional;
    }

  }

  /**
   * @returns The Optional Translation2d point used for targeting.
   */
  public Optional<Translation2d> getFieldTarget() {
    return fieldTarget;
  }

  public void configurePathPlanner() {

    AutoBuilder.configureHolonomic(this::getRobotPose, this::resetOdometry,
        this::getRobotRelativeSpeeds, this::setDriveChassisSpeed,
        new HolonomicPathFollowerConfig(
            new PIDConstants(Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KP,
                Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KI,
                Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KD),
            new PIDConstants(Constants.PathPlannerInfo.PATHPLANNER_TURN_KP,
                Constants.PathPlannerInfo.PATHPLANNER_TURN_KI,
                Constants.PathPlannerInfo.PATHPLANNER_TURN_KD),
            Constants.PathPlannerInfo.PATHPLANNER_MAX_METERS_PER_SECOND,
            Constants.PathPlannerInfo.PATHPLANNER_DRIVEBASE_RADIUS_METERS, new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this);

  }

  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }

  public Command followAutoTrajectory(String autoName) {
    return new PathPlannerAuto(autoName);
  }
}
