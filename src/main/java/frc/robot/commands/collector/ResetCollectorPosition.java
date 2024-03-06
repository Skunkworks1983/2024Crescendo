// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.Collector.LimitSwitch;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ResetCollectorPosition extends Command {

  Collector collector;
  
  public ResetCollectorPosition() {
    collector = Collector.getInstance();
    addRequirements(SubsystemGroups.getInstance(Subsystems.COLLECTOR_PIVOT));
  }

  @Override
  public void initialize() {
    collector.setPivotPercentOutput(.1);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    collector.resetPivotEncoderPosition();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
