// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.PhotonVision;
import frc.robot.vision.PieceData;
import frc.robot.vision.Vision;

public class CenterOnPiece extends Command {

  Vision vision = Vision.getInstance();

  PIDController centeringController = new PIDController(0, 0, 0);

  public CenterOnPiece() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<PieceData> pieceData = vision.getPieceData(PhotonVision.PIECE_DETECTION_CAMERA_NAME);
    if (pieceData.isPresent()) {
      centeringController.setSetpoint(pieceData.get().yaw);
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
