// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.Optional;

import javax.swing.TransferHandler.TransferSupport;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.PhotonVision;
import frc.robot.utils.Vision;

public class CameraStdDevsTuning extends Command {

  Vision vision;
  LinkedList<Transform3d> measurements;

  public CameraStdDevsTuning() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Optional<Transform3d> transformOptional = vision.getRobotToTargetTransform(PhotonVision.CAMERA_1_NAME);

    if (transformOptional.isPresent()) {
      Transform3d transform = transformOptional.get();
      measurements.add(transform);
    }

    // Calculate the average x, y, z, and rotational components of the list of measurments.
    double xTotal = 0;
    double yTotal = 0;
    double rotTotal = 0;

    for (Transform3d transform3d : measurements) {
      xTotal += transform3d.getX();
      yTotal += transform3d.getY();
      rotTotal += transform3d.getZ();
    }

    double xAverage = xTotal / measurements.size();
    double yAverage = yTotal / measurements.size();
    double rotAverage = rotTotal / measurements.size();

    // Use the averages to calculate the standard deviations.
    // double xStdDev = Math.sqrt(/* code to calculate std dev */);
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
