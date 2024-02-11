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
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SwerveTeleop;
import frc.robot.constants.Constants;
import frc.robot.utils.SmartPIDController;

public class Drivebase extends SubsystemBase {

  private static Drivebase drivebase;
  OI oi = OI.getInstance();
  AHRS gyro = new AHRS(I2C.Port.kOnboard);
  private final Field2d integrated = new Field2d();
  private final Field2d visual = new Field2d();
  PhotonCamera camera = new PhotonCamera(Constants.PHOTON_CAMERA_NAME);
  ChassisSpeeds speeds;
  Pose2d pose;
  AprilTagFieldLayout aprilTagFieldLayout;
  SmartPIDController headingController = new SmartPIDController(
    Constants.PIDControllers.HeadingControlPID.KP, 
    Constants.PIDControllers.HeadingControlPID.KI, 
    Constants.PIDControllers.HeadingControlPID.KD,
    "Heading Controller",
    Constants.PIDControllers.HeadingControlPID.SMART_PID_ACTIVE
  );

  // locations of the modules, x positive forward y positive left
  Translation2d leftFrontLocation = new Translation2d(
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y)
  );

  Translation2d rightFrontLocation = new Translation2d(
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y)
  );

  Translation2d leftBackLocation = new Translation2d(
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(Constants.DrivebaseInfo.TRANSLATION_Y)
  );

  Translation2d rightBackLocation = new Translation2d(
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_X),
    Units.feetToMeters(-Constants.DrivebaseInfo.TRANSLATION_Y)
  );

  SwerveModule frontLeft = new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.FRONT_LEFT_MODULE);

  SwerveModule frontRight = new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.FRONT_RIGHT_MODULE);

  SwerveModule backLeft = new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.BACK_LEFT_MODULE);

  SwerveModule backRight = new SwerveModule(Constants.DrivebaseInfo.ModuleConstants.BACK_RIGHT_MODULE);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    leftFrontLocation, 
    rightFrontLocation, 
    leftBackLocation, 
    rightBackLocation
  );

  SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
      kinematics,
      Rotation2d.fromDegrees(getGyroAngle()),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()},
      new Pose2d(0, 0, Rotation2d.fromDegrees(0))); 

  PhotonPoseEstimator visualOdometry;

  private Drivebase() {
    gyro.reset();
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
      AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("Exception reading AprilTag Field JSON " + e.toString());
    }
    visualOdometry = new PhotonPoseEstimator(
      aprilTagFieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      camera, 
      Constants.ROBOT_TO_CAMERA);
    resetOdometry(Constants.BLUE_START_POS);
    
    SmartDashboard.putData("Integrated Odom", integrated);
    SmartDashboard.putData("Visual Odom", visual);
    configurePathPlanner();

    headingController.enableContinuousInput(0, 360);

    SmartDashboard.putNumber("testTurnP",0);
    SmartDashboard.putNumber("testTurnI",0);
    SmartDashboard.putNumber("testTurnD",0);
  }

  /** run in teleop init to set swerve as default teleop command */
  public void setSwerveAsDefaultCommand() {
    setDefaultCommand(new SwerveTeleop(drivebase, oi));
  }

  // returns angle going counterclockwise
  public double getGyroAngle() {
    double angle = gyro.getAngle();
    SmartDashboard.putNumber("gyro angle", angle);
    return -angle;
    // negative because gyro reads differently than wpilib
  }
  public void setDrive(double xFeetPerSecond, double yFeetPerSecond, double degreesPerSecond, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        Units.feetToMeters(xFeetPerSecond),
        Units.feetToMeters(yFeetPerSecond),
        Units.degreesToRadians(degreesPerSecond),
        Rotation2d.fromDegrees(getGyroAngle())
      ); 
    } else {
      speeds = new ChassisSpeeds(
        Units.feetToMeters(xFeetPerSecond),
        Units.feetToMeters(yFeetPerSecond),
        Units.degreesToRadians(degreesPerSecond)
      );
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    
    // sets module max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED);
    setModuleStates(moduleStates);
  }

  //used for keeping robot heading in the right direction using PID and the targeting buttion 
  public void setDriveTurnPos(double xFeetPerSecond, double yFeetPerSecond, boolean fieldRelative) {

    double degreesPerSecond;
    degreesPerSecond = headingController.calculate(getGyroAngle());

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

  public Pose2d getRobotPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d resetPose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(getGyroAngle()), 
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      },
      resetPose
    );
  }

  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getGyroAngle()),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
    );
    Optional<EstimatedRobotPose> updated = visualOdometry.update();
    PhotonPipelineResult result = camera.getLatestResult();
    SmartDashboard.putBoolean("Targets found", result.hasTargets());
    if (updated.isPresent() && result.hasTargets()) {
      Transform3d distanceTransform = result.getBestTarget().getBestCameraToTarget();
      EstimatedRobotPose pose = updated.get();
      double distance = Math.sqrt(Math.pow(distanceTransform.getX(), 2) + Math.pow(distanceTransform.getY(), 2));
      SmartDashboard.putNumber("Distance to target", distance);
      Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(
        new SimpleMatrix(
          new double [] {
            distance * Constants.DISTANCE_UNCERTAINTY,
            distance * Constants.DISTANCE_UNCERTAINTY,
            9999999                                      // gyro is better, use gyro instead
          }
        )
      );
      visual.setRobotPose(pose.estimatedPose.toPose2d());
      odometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, uncertainty);
      SmartDashboard.putNumber("vision uncertainty", distance * Constants.DISTANCE_UNCERTAINTY);
    }
    integrated.setRobotPose(odometry.getEstimatedPosition()); 
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
  
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
      frontLeft.getSwerveState(),
      frontRight.getSwerveState(),
      backLeft.getSwerveState(),
      backRight.getSwerveState()
    );

    return chassisSpeeds;
}

public void setDriveChassisSpeed(ChassisSpeeds chassisSpeeds) {
  setDrive(
    Units.metersToFeet(chassisSpeeds.vxMetersPerSecond),
    Units.metersToFeet(chassisSpeeds.vyMetersPerSecond),
    Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond),
    //path planner uses robot reletive drive command.
    false
  );
}

public void configurePathPlanner() {

  AutoBuilder.configureHolonomic(
    this::getRobotPose,
    this::resetOdometry,
    this::getRobotRelativeSpeeds, 
    this::setDriveChassisSpeed,
    new HolonomicPathFollowerConfig(
      new PIDConstants(
        Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KP, 
        Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KI, 
        Constants.PathPlannerInfo.PATHPLANNER_DRIVE_KD
      ),
      new PIDConstants(
        Constants.PathPlannerInfo.PATHPLANNER_TURN_KP, 
        Constants.PathPlannerInfo.PATHPLANNER_TURN_KI, 
        Constants.PathPlannerInfo.PATHPLANNER_TURN_KD
      ),
      Constants.PathPlannerInfo.PATHPLANNER_MAX_METERS_PER_SECOND,
      Constants.PathPlannerInfo.PATHPLANNER_DRIVEBASE_RADIUS_METERS,
      new ReplanningConfig()
    ), 
    () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    }, 
    this
  );

}
  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return AutoBuilder.followPath(path);
  }

  public Command followAutoTrajectory(String autoName) {
    return new PathPlannerAuto(autoName);
  }
}