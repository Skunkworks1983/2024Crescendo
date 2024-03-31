// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test_commands;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class FindBruteForceWorkingVisionPositions extends Command {

  Drivebase drivebase;
  Pose2d actualPose;
  ArrayList<Double> goodIndexes = new ArrayList<>();// is double so smartdashboard can accept
  Pose2d[] error;
  double[] inputX;
  double[] inputPitch;
  double[] inputYaw;
  int timesRun = 0;

  /** Creates a new FindBruteForceWorkingVisionPositions. */
  public FindBruteForceWorkingVisionPositions() {
    SmartDashboard.putNumber("visualOdometryXPositionActual", 0.0);
    SmartDashboard.putNumber("visualOdometryYPositionActual", 0.0);
    SmartDashboard.putNumber("timeToTakeAverage", 0.02 * 5.0);// 5 frames
    drivebase = Drivebase.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    actualPose = new Pose2d(
        new Translation2d(SmartDashboard.getNumber("visualOdometryXPositionActual", 0),
            SmartDashboard.getNumber("visualOdometryXPositionActual", 0)),
        Rotation2d.fromDegrees(SmartDashboard.getNumber("visualOdometryThetaRotationActual", 0)));

    int xLoops = 50;
    double xVarience = .5;
    int pitchLoops = 10;
    double pitchLow = 9;
    double pitchHigh = 17;

    int yawLoops = 10;
    double yawVarience = 4;// in each dirrection
    int total = xLoops * pitchLoops * yawLoops;
    error = new Pose2d[total];
    inputX = new double[total];
    inputPitch = new double[total];
    inputYaw = new double[total];

    int i = 0;
    for (double x = -xVarience; x <= xVarience; x += xVarience * 2 / xLoops) {
      for (double pitch = pitchLow; pitch <= pitchHigh; pitch +=
          (pitchHigh - pitchLow) / pitchLoops) {
        for (double yaw = -yawVarience; yaw <= xVarience; yaw += yawVarience * 2 / yawLoops) {
          inputX[i] = x;
          inputPitch[i] = pitch;
          inputYaw[i] = yaw;
        }
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {// x pitch and yaw
    Transform3d t = new Transform3d(inputX[timesRun], Units.inchesToMeters(0.901 + .875),
        Units.inchesToMeters(10.727 + 2.088 - 0.175),
        new Rotation3d(0, inputPitch[timesRun], inputYaw[timesRun]));
    Pose2d percivedPose = drivebase.setVisionOffsetCameraOne(t);
    if(percivedPose==null)return;
    double diffX = actualPose.getX() - percivedPose.getX();
    double diffY = actualPose.getY() - percivedPose.getY();
    double diffTheta =
        actualPose.getRotation().getDegrees() - percivedPose.getRotation().getDegrees();
    error[timesRun] =
        new Pose2d(new Translation2d(diffX, diffY), Rotation2d.fromDegrees(diffTheta));
    if (error[timesRun].getTranslation().getNorm() < .04) {
      System.out.println("Good value at" + timesRun + " : " + inputX[timesRun] + ", "
          + inputPitch[timesRun] + ", " + inputYaw[timesRun]);
      goodIndexes.add(new Double(timesRun));
    }
    timesRun++;
    SmartDashboard.putNumberArray("good indexes",
        DoubletodoubleArray((Double[]) goodIndexes.toArray()));
  }

  double[] DoubletodoubleArray(Double[] a) {
    int i = 0;
    double[] b = new double[a.length];
    for (Double c : a) {
      b[i] = c.doubleValue();
      i++;
    }
    return b;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double lowestValue=100000;
    int lowestIndex=0;
    int i = 0;
    for(var a : error){
      i++;
      if(a.getTranslation().getNorm()<lowestValue){
        lowestValue = a.getTranslation().getNorm();
        lowestIndex = i;
      };
      System.out.println("Best value" + lowestIndex + " : " + inputX[lowestIndex] + ", "
      + inputPitch[lowestIndex] + ", " + inputYaw[lowestIndex]);
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (error.length <= timesRun);// will skip final one I think but its fine
    // return false;
  }
}
