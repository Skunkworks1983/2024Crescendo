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
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SwerveTeleop;
import frc.robot.constants.Constants;

public class Drivebase extends SubsystemBase {

  private static Drivebase drivebase;
  OI oi = OI.getInstance();
  AHRS gyro = new AHRS(I2C.Port.kOnboard);
  private final Field2d odometryFieldPos = new Field2d();
  PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  ChassisSpeeds speeds;
  Pose2d pose;
  AprilTagFieldLayout aprilTagFieldLayout;
  PIDController headingController = new PIDController(
    Constants.PIDControllers.HeadingControlPID.KP, 
    Constants.PIDControllers.HeadingControlPID.KI, 
    Constants.PIDControllers.HeadingControlPID.KD
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


  SwerveModule frontLeft = new SwerveModule(
    Constants.IDS.LEFT_FRONT_DRIVE, Constants.IDS.LEFT_FRONT_TURN,
    Constants.IDS.LEFT_FRONT_CAN_CODER, Constants.DrivebaseInfo.FRONT_LEFT_OFFSET
  );

  SwerveModule frontRight = new SwerveModule(
    Constants.IDS.RIGHT_FRONT_DRIVE, Constants.IDS.RIGHT_FRONT_TURN,
    Constants.IDS.RIGHT_FRONT_CAN_CODER, Constants.DrivebaseInfo.FRONT_RIGHT_OFFSET
  );

  SwerveModule backLeft = new SwerveModule(
    Constants.IDS.LEFT_BACK_DRIVE, Constants.IDS.LEFT_BACK_TURN,
    Constants.IDS.LEFT_BACK_CAN_CODER, Constants.DrivebaseInfo.BACK_LEFT_OFFSET
  );

  SwerveModule backRight = new SwerveModule(
    Constants.IDS.RIGHT_BACK_DRIVE, Constants.IDS.RIGHT_BACK_TURN,
    Constants.IDS.RIGHT_BACK_CAN_CODER, Constants.DrivebaseInfo.BACK_RIGHT_OFFSET
  );

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    leftFrontLocation, 
    rightFrontLocation, 
    leftBackLocation, 
    rightBackLocation
  );

  SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
      kinematics,
      Rotation2d.fromDegrees(-getGyroAngle()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()},
      new Pose2d(0, 0, Rotation2d.fromDegrees(0))); 

  PhotonPoseEstimator visualOdometry;

  private Drivebase() {
    gyro.reset();
    //resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    SmartDashboard.putData("Field Pos", odometryFieldPos);
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
      AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("exception reading field json " + e.toString());
    }
    visualOdometry = new PhotonPoseEstimator(
    aprilTagFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
    camera, 
    Constants.CAMERA_TO_ROBOT);
  }

  /** run in teleop init to set swerve as default teleop command */
  public void setSwerveAsDefaultCommand() {
    setDefaultCommand(new SwerveTeleop(drivebase, oi));
  }

  //returns angle going counterclockwise
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
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED); // sets module max speed

    setModuleStates(moduleStates);
  }

  //used for when not using rotation, keeps robot heading in the right direction using PID
  public void setDriveDeadband(double xFeetPerSecond, double yFeetPerSecond, boolean fieldRelative) {

    double degreesPerSecond;
    degreesPerSecond = headingController.calculate(-getGyroAngle());


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
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_MODULE_SPEED);

    setModuleStates(moduleStates);
  }

  public void setHeadingController(double setpoint){
    headingController.setSetpoint(setpoint);
  }


  public void setModuleStates(SwerveModuleState[] states) {
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    backLeft.setState(states[2]);
    backRight.setState(states[3]);
  } 

  public Pose2d getOdometry() {
    return odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d resetPose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(-getGyroAngle()), 
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
    odometry.update(Rotation2d.fromDegrees(-getGyroAngle()),
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      }
    );
    Optional<EstimatedRobotPose> updatedVisualOdometry = visualOdometry.update();
    PhotonPipelineResult result = camera.getLatestResult();
    SmartDashboard.putBoolean("Targets found", result.hasTargets());
    SmartDashboard.putBoolean("Block running", updatedVisualOdometry.isPresent());
    if (updatedVisualOdometry.isPresent()) {
      EstimatedRobotPose pose = updatedVisualOdometry.get();
      Transform3d distanceToTarget = result.getBestTarget().getBestCameraToTarget();
      double distance = Math.sqrt(Math.pow(distanceToTarget.getX(), 2) + Math.pow(distanceToTarget.getY(), 2));
      SmartDashboard.putNumber("Distance to target", distance);
      Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(
        new SimpleMatrix(
          new double [] {
            distance * Constants.DISTANCE_UNCERTAINTY,
            distance * Constants.DISTANCE_UNCERTAINTY,
            999999                                               // gyro is better, use gyro instead
          }
        )
      );
      odometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, uncertainty);
    }
    odometryFieldPos.setRobotPose(odometry.getEstimatedPosition());
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
}