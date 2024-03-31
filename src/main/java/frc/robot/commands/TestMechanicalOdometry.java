// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class TestMechanicalOdometry extends Command {

  Drivebase drivebase;
  Pose2d startPose;
  Pose2d endPose;

  /** Debug purposes only */
  public TestMechanicalOdometry() {
    drivebase = Drivebase.getInstance();
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    drivebase.zeroModulesAndSetCoastMode();
    startPose = drivebase.getRobotPose();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    endPose = drivebase.getRobotPose();

    System.out.println("CHANGE IN ODOMETRY POSITION:");
    System.out.println("X meters: " + (endPose.getX() - startPose.getX()));
    System.out.println("Y meters: " + (endPose.getY() - startPose.getY()));
    System.out.println("Rot degrees: "
        + (endPose.getRotation().getDegrees() - startPose.getRotation().getDegrees()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
