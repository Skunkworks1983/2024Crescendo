// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.Optional;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.PhotonVision;
import frc.robot.utils.Vision;

public class CameraStdDevsTuning extends Command {

  Vision vision;
  LinkedList<Transform3d> measurements;
  LinkedList<Double> xMeasurements;
  LinkedList<Double> yMeasurements;
  LinkedList<Double> rotMeasurements;

  File logFile;
  FileWriter fileWriter;

  public CameraStdDevsTuning() {}

  public double calculateStdDevs(LinkedList<Double> measurements) {
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
    // try {
    // logFile = new File("Camera_Std_Devs_Tuning_Log.txt");
    // fileWriter = new FileWriter("Camera_Std_Devs_Tuning_Log.txt");
    // fileWriter.write("COMMAND INITIALIZE");

    // if (logFile.createNewFile()) {
    // System.out.println("Successfully created file");
    // } else {
    // System.out.println("File already exists");
    // }
    // } catch (IOException e) {
    // e.printStackTrace();
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Optional<Transform3d> transformOptional =
        vision.getRobotToTargetTransform(PhotonVision.CAMERA_1_NAME);

    if (transformOptional.isPresent()) {
      Transform3d transform3d = transformOptional.get();
      measurements.add(transform3d);
    }

    for (Transform3d measurement : measurements) {
      xMeasurements.add(measurement.getX());
      yMeasurements.add(measurement.getY());
      rotMeasurements.add(measurement.getRotation().toRotation2d().getDegrees());

      SmartDashboard.putNumber("Camera Tranform X", measurement.getX());
      SmartDashboard.putNumber("Camera Transform Y", measurement.getY());
      SmartDashboard.putNumber("Camera Transform Rot", measurement.getRotation().toRotation2d().getDegrees());
    }

    if (measurements.size() > PhotonVision.STD_DEVS_TUNING_MEASUREMENTS_LIST_SIZE) {
      measurements.remove(0);
    }

    if (xMeasurements.size() > PhotonVision.STD_DEVS_TUNING_MEASUREMENTS_LIST_SIZE) {
      xMeasurements.remove(0);
    }

    if (yMeasurements.size() > PhotonVision.STD_DEVS_TUNING_MEASUREMENTS_LIST_SIZE) {
      yMeasurements.remove(0);
    }

    if (rotMeasurements.size() > PhotonVision.STD_DEVS_TUNING_MEASUREMENTS_LIST_SIZE) {
      rotMeasurements.remove(0);
    }

    double xStdDev = calculateStdDevs(xMeasurements);
    double yStdDev = calculateStdDevs(yMeasurements);
    double rotStdDev = calculateStdDevs(rotMeasurements);

    SmartDashboard.putNumber("Camera X Std Dev", xStdDev);
    SmartDashboard.putNumber("Camera Y Std Dev", yStdDev);
    SmartDashboard.putNumber("Camera Rotation Std Dev", rotStdDev);

    // Write to the file
    // try {
    // fileWriter.write("Timestamp: " + Timer.getFPGATimestamp());
    // fileWriter.write("X : " + Double.toString(xStdDev));
    // fileWriter.write("Y : " + Double.toString(yStdDev));
    // fileWriter.write("Rot: " + Double.toString(rotStdDev));
    // fileWriter.close();
    // } catch (IOException e) {
    // System.out.println("Execption writing to file: " + e.toString());
    // }
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
