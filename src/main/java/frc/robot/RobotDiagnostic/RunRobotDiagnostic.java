// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotDiagnostic;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.WaitDuration;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunRobotDiagnostic extends ParallelDeadlineGroup {
  /** Creates a new RunRobotDiagnostic. */
  public RunRobotDiagnostic() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitDuration(3.0));
    addCommands(new DrivebaseDiagnostic(), new ClimberDiagnostic(), new PrintDiagnosticResults());
  }
}
