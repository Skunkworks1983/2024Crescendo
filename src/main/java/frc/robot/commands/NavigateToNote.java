// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PhotonVision;
import frc.robot.subsystems.Drivebase;
import frc.robot.vision.PieceData;
import frc.robot.vision.Vision;

public class NavigateToNote extends Command {

  Vision vision;
  Drivebase drivebase;
  Command pathToNote;

  public NavigateToNote() {
    vision = Vision.getInstance();
    drivebase = Drivebase.getInstance();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // Get piece data from the camera
    Optional<PieceData> pieceData = vision.getPieceData(PhotonVision.PIECE_DETECTION_CAMERA_NAME);

    // Check if there is a piece in view
    if (pieceData.isPresent()) {

      // Get a transform to the piece
      Transform3d robotToPiece = PhotonVision.ROBOT_TO_PIECE_DETECTION_CAMERA
          .plus(pieceData.get().bestCameraToTargetTransform);

      Pose2d noteFieldPosition = drivebase.getRobotPose()
          .transformBy(new Transform2d(new Pose2d(),
              new Pose2d(robotToPiece.getX(), robotToPiece.getY(), robotToPiece.getRotation().toRotation2d())));

      pathToNote = drivebase.pathfindToPose(noteFieldPosition);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
