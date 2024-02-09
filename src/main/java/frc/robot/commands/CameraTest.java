// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class CameraTest extends Command {

  PhotonPoseEstimator visualOdometry;
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonCamera camera = new PhotonCamera(Constants.PHOTON_CAMERA_NAME);
  private final Field2d visual = new Field2d();
  ArrayList<Pose2d> rollingAverage = new ArrayList<>();
  int rollingAverageLength = 50;

  public CameraTest() {
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
    
    SmartDashboard.putData("Visual Odom", visual);
    SmartDashboard.putBoolean("code running", true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  Pose2d updateRollingAverage(Pose2d latestPose) {
    rollingAverage.add(latestPose);
      if (rollingAverage.size() > rollingAverageLength) {
        rollingAverage.remove(0);
      }
      double xTotal = 0;
      double yTotal = 0;
      double rotAbsTotal = 0;
      double rotTotal = 0;
      for(int i = 0; i < rollingAverage.size()-1; i++) {
        xTotal = xTotal + rollingAverage.get(i).getX();
        yTotal = yTotal + rollingAverage.get(i).getY();
        rotTotal = rotTotal + rollingAverage
      }

      double average = rotTotal1/rollingAverageLength;
      if ((average))

      Pose2d averagedPose = new Pose2d(
        xTotal/rollingAverageLength, 
        yTotal/rollingAverageLength, 
        latestPose.getRotation());

      return averagedPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> updatedVisualOdometry = visualOdometry.update();
    PhotonPipelineResult result = camera.getLatestResult();
    SmartDashboard.putBoolean("Targets found", result.hasTargets());
    SmartDashboard.putBoolean("Block running", updatedVisualOdometry.isPresent());
    if (updatedVisualOdometry.isPresent()) {
      EstimatedRobotPose pose = updatedVisualOdometry.get();
      Transform3d distanceToTarget = result.getBestTarget().getBestCameraToTarget();
      SmartDashboard.putNumber("x distance to target", Units.metersToFeet(distanceToTarget.getX()));
      double distance = Math.sqrt(Math.pow(distanceToTarget.getX(), 2) + Math.pow(distanceToTarget.getY(), 2));
      SmartDashboard.putNumber("Distance to target", Units.metersToFeet(distance));

      // Matrix<N3, N1> uncertainty = new Matrix<N3, N1>(
      //   new SimpleMatrix(
      //     new double [] {
      //       distance * Constants.DISTANCE_UNCERTAINTY,
      //       distance * Constants.DISTANCE_UNCERTAINTY,
      //       0                                     // gyro is better, use gyro instead
      //     }
      //   )
      // );
      

      
      visual.setRobotPose(updateRollingAverage(pose.estimatedPose.toPose2d()));
      // SmartDashboard.putNumber("X", Units.metersToFeet(pose.estimatedPose.toPose2d().getX()));
      // SmartDashboard.putNumber("Y", Units.metersToFeet(pose.estimatedPose.toPose2d().getY()));
      // SmartDashboard.putNumber("angle", pose.estimatedPose.toPose2d().getRotation().getDegrees());
      // SmartDashboard.putNumber("vision uncertainty", distance * Constants.DISTANCE_UNCERTAINTY);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
