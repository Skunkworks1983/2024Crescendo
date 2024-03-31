// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SwerveTeleop;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Targeting.FieldTarget;
import frc.robot.utils.SkunkPhotonCamera;
import frc.robot.utils.SmartPIDController;
import frc.robot.utils.Vision;
import frc.robot.utils.VisionMeasurement;
import frc.robot.constants.Constants.PhotonVision;

public class Drivebase extends SubsystemBase {

  private static Drivebase drivebase;

  AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

  // Shuffleboard/Glass visualizations of robot position on the field.
  private final Field2d integratedOdometryPrint = new Field2d();
  private final Field2d visualOdometryPrint = new Field2d();

  ChassisSpeeds speeds;
  public boolean isRobotRelative;

  // Position used for targeting.
  Optional<Translation2d> fieldTarget;

  double maxVelocity = 0;
  double gyroOffset;
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

  Vision vision;

  private Drivebase() {
    isRobotRelative = false;
    // The robot should have the same heading as the heading specified here on
    // startup.
    resetOdometry(new Pose2d(Constants.FIELD_X_LENGTH / 2, Constants.FIELD_Y_LENGTH / 2,
        Rotation2d.fromDegrees(0)));

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
    gyro.reset();

    // Try/catch statement to ensure robot code doesn't crash if camera(s) aren't
    // plugged in.
    try {
      vision = new Vision(new SkunkPhotonCamera[] {
          new SkunkPhotonCamera(PhotonVision.CAMERA_1_NAME, PhotonVision.ROBOT_TO_CAMERA_1),
          new SkunkPhotonCamera(PhotonVision.CAMERA_2_NAME, PhotonVision.ROBOT_TO_CAMERA_2)});
      SmartDashboard.putBoolean(PhotonVision.CAMERA_STATUS_BOOLEAN, true);
    } catch (Exception e) {
      System.out.println("Exception creating cameras: " + e.toString());
      vision = new Vision(new SkunkPhotonCamera[] {});
      SmartDashboard.putBoolean(PhotonVision.CAMERA_STATUS_BOOLEAN, false);
    }

    resetGyroOffset();
  }

  /** run in teleop init to set swerve as default teleop command */
  public void setSwerveAsDefaultCommand() {
    setDefaultCommand(new SwerveTeleop(drivebase, OI.getInstance()));
  }

  /** Get an instance of the gyro system for use in commands. */

public Pose2d setVisionOffsetCameraOne(Transform3d t){
    vision = new Vision(new SkunkPhotonCamera[] {
      new SkunkPhotonCamera(PhotonVision.CAMERA_1_NAME, t),
      new SkunkPhotonCamera(PhotonVision.CAMERA_2_NAME, PhotonVision.ROBOT_TO_CAMERA_2)});
    SmartDashboard.putBoolean(PhotonVision.CAMERA_STATUS_BOOLEAN, true);
    var a=vision.getLatestVisionMeasurements();
    if(a.size() == 0)return null;
    return a.get(0).estimatedPose;
  }

  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond,
      boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond), Units.degreesToRadians(degreesPerSecond),
          Rotation2d.fromDegrees(getGyroAngle()));
    } else {
      speeds = new ChassisSpeeds(Units.feetToMeters(xFeetPerSecond),
          Units.feetToMeters(yFeetPerSecond), Units.degreesToRadians(degreesPerSecond));
    }
    double Cx = speeds.vxMetersPerSecond;
    double Cy = speeds.vyMetersPerSecond;
    double Omega = speeds.omegaRadiansPerSecond;
    double magnitudeSpeed = Math.sqrt(Math.pow(Cx, 2) + Math.pow(Cy, 2));
    double K = Constants.DrivebaseInfo.CORRECTIVE_SCALE;

    speeds.vxMetersPerSecond =
        Cx + K * Omega * Math.sin(-Math.atan2(Cx, Cy) + Math.PI / 2) * magnitudeSpeed;
    speeds.vyMetersPerSecond =
        Cy - K * Omega * Math.cos(-Math.atan2(Cx, Cy) + Math.PI / 2) * magnitudeSpeed;

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Caps the module speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED);
    setModuleStates(moduleStates);
  }

  // Used for keeping robot heading in the right direction using PID and the
  // targeting buttion
  public void setDriveTurnPos(double xFeetPerSecond, double yFeetPerSecond, boolean fieldRelative) {
    double degreesPerSecond;
    degreesPerSecond = Math.min(Constants.TURNING_SPEED_CAP,
        Math.max(-Constants.TURNING_SPEED_CAP, headingController.calculate(getGyroAngle())));
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

  /**
   * Call this method instead of getGyroAngle(). This method returns the robot's heading according
   * to the integrated odometry. This allows for an accurate heading measurement, even if the gyro
   * is inaccurate.
   * 
   * @return The heading of the robot according to the integrated odometry, in degrees. Note:
   *         Measurement is 0-360 degrees instead of continuous.
   */
  public double getRobotHeading() {
    if (!isRobotRelative) {
      return getRobotPose().getRotation().getDegrees();
    }

    return 0.0;
  }

  public void setRobotRelative() {
    isRobotRelative = true;
  }

  public void setFieldRelative() {
    isRobotRelative = false;
  }

  public boolean getFieldRelative() {
    return !isRobotRelative;
  }

  public double getGyroAngle() {
    return -gyro.getAngle() - gyroOffset;
  }

  public void resetGyroOffset() {
    gyroOffset = -gyro.getAngle();
  }

  public double getRoll() {
    return gyro.getRoll();
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

    // Iterate though list of VisionMeasurements and call addVisionMeasurement for
    // each item in the list.
    for (VisionMeasurement measurement : vision.getLatestVisionMeasurements()) {
      odometry.addVisionMeasurement(measurement.estimatedPose, measurement.timestamp,
          measurement.stdDevs);
    }

    integratedOdometryPrint.setRobotPose(getRobotPose());
  }

  @Override
  public void periodic() {
    updateOdometry();
    SmartDashboard.putNumber("Odometry X Meters", odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Odometry Y Meters", odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putBoolean("is Robot Relative", isRobotRelative);
    SmartDashboard.putNumber("Heading Controller Error", headingController.getPositionError());
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

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),
        Rotation2d.fromDegrees(getGyroAngle()));
  }

  public void setDriveChassisSpeed(ChassisSpeeds chassisSpeeds) {
    setDrive(Units.metersToFeet(chassisSpeeds.vxMetersPerSecond),
        Units.metersToFeet(chassisSpeeds.vyMetersPerSecond),
        Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond),
        // path planner uses robot reletive drive command.
        false);
  }

  // sets the back right module pos for tuning
  public void setBackRightModuleTurnPos(double angleDegrees) {
    backRight.setTurnMotorAngle(angleDegrees);
  }

  // returning the back right turn pos for tunning
  public double getBackRightModuleTurnPos() {
    return backRight.getSwerveState().angle.getDegrees();
  }

  // returning the back right turn error for tunning
  public double getBackRightModuleTurnError() {
    return backRight.getTurnError();
  }

  // returning the back right turn speed for tunning
  public double getBackRightModuleTurnVelocity() {
    return backRight.getTurnMotorVelocity();
  }

  /**
   * Sets the current target point used for targeting.
   * 
   * @param target The target to point at (FieldTarget enum value)
   */
  public void setFieldTarget(FieldTarget fieldTarget) {

    Optional<Translation2d> fieldTargetOptional;

    if (fieldTarget == null || fieldTarget == FieldTarget.NONE) {
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
