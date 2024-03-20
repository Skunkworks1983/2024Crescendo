// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.PhotonVision;
import frc.robot.utils.Vision;

public class CameraStdDevsTuning extends Command {

  Vision vision;
  LinkedList<Transform3d> measurements;

  public CameraStdDevsTuning() {
  }

  public double calculateStdDevs(Supplier<Double> measurementSupplier) {
    double averageTotal = 0;


    for (double measurement : measurements) {
      averageTotal += measurement;
    }

    double average = averageTotal / measurements.size();

    double numeratorTotal = 0;

    for (double measurement : measurements) {
      double difference = measurement - average;
      numeratorTotal += Math.pow(difference, 2);
    }

    double stdDev = Math.sqrt(numeratorTotal / measurements.size());

    return stdDev;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Optional<Transform3d> transformOptional = vision.getRobotToTargetTransform(PhotonVision.CAMERA_1_NAME);

    if (transformOptional.isPresent()) {
      Transform3d transform3d = transformOptional.get();
      measurements.add(transform3d);
    }

    for (Transform3d measurement : measurements) {

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
