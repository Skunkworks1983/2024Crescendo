// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PhotonVision;
import frc.robot.vision.PieceData;
import frc.robot.vision.Vision;

public class NavigateToNote extends Command {

  Vision vision;

  public NavigateToNote() {

    vision = Vision.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<PieceData> pieceData = vision.getPieceData(Constants.PhotonVision.PIECE_DETECTION_CAMERA_NAME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<PieceData> pieceData = vision.getPieceData(PhotonVision.PIECE_DETECTION_CAMERA_NAME);

    if (pieceData.isPresent()) {
        Transform3d robotToPieceTransform = PhotonVision.ROBOT_TO_PIECE_DETECTION_CAMERA.plus(pieceData.get().bestCameraToTargetTransform);
        
    } else 
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
