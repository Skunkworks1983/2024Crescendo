// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebaseTeleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class SwerveTeleop extends Command {

  Drivebase drivebase;
  OI oi;

  public SwerveTeleop() {
    drivebase = Drivebase.getInstance();
    oi = OI.getInstance();
    addRequirements(drivebase);
  }

  public ChassisSpeeds getChassisSpeedsUsingJoysticks () {
    return new ChassisSpeeds();
  }
}
