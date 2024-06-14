// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.WaitDuration;

public class RunRobotDiagnostic extends ParallelDeadlineGroup {

  public RunRobotDiagnostic() {

    super(new WaitDuration(1.0));
    addCommands(new DrivebaseDiagnostic(), new ClimberDiagnostic(), new PrintDiagnostic());
  }
}
