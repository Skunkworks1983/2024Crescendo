// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CameraTest extends Command {

  PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  PhotonPipelineResult result;
  PhotonTrackedTarget target;
  Transform3d pose;

  public CameraTest() {
    SmartDashboard.putBoolean("constructor", true);
    SmartDashboard.putBoolean("target", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = camera.getLatestResult();
    if (result.hasTargets()) {
      target = result.getBestTarget();
      pose = target.getBestCameraToTarget();
      SmartDashboard.putNumber("x distance", pose.getX());
      SmartDashboard.putNumber("y distance", pose.getY());
      SmartDashboard.putNumber("z distance", pose.getZ());
      SmartDashboard.putBoolean("target", true);
    } else {
      SmartDashboard.putBoolean("target", false);
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
