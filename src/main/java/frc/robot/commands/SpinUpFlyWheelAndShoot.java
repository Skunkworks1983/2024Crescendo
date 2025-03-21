// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.shooter.FlywheelSpinup;
import frc.robot.commands.shooter.ShootWhenReady;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpinUpFlyWheelAndShoot extends ParallelDeadlineGroup {
  /** Creates a new SpinUpFlyWheelAndShoot. */
  public SpinUpFlyWheelAndShoot() {
    super ((Command)new ShootWhenReady(), new FlywheelSpinup());
  }
}
