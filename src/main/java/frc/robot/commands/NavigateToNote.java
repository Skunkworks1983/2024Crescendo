// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.vision.PieceData;
import frc.robot.vision.Vision;
import frc.robot.constants.Constants.PhotonVision;

public class NavigateToNote {

  Vision vision;
  Drivebase drivebase;
  Command pathToNote;
  Pose2d lastestNotePose;

  public NavigateToNote(Pose2d defaultNotePosition) {
    vision = Vision.getInstance();
    drivebase = Drivebase.getInstance();

    this.lastestNotePose = defaultNotePosition;
  }

  public Command getNavigateToNoteCommand() {

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

      lastestNotePose = noteFieldPosition;

      return drivebase.pathfindToPose(lastestNotePose);
    }

    return pathToNote;
  }
}
