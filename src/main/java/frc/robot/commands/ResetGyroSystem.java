// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GyroSystem;
import frc.robot.subsystems.Drivebase;

/**
 * The reset gyro button must be pressed on both joysticks simaltenously in order to reset the
 * robots heading. This makes it more challenging to reset the robot's heading by accident.
 */
public class ResetGyroSystem extends InstantCommand {

  Drivebase drivebase;
  GyroSystem gyroSystem;
  Supplier<Boolean> isLeftButtonPressed, isRightButtonPressed;

  public ResetGyroSystem(Supplier<Boolean> isLeftButtonPressed, Supplier<Boolean> isRightButtonPressed) {
    this.isLeftButtonPressed = isLeftButtonPressed;
    this.isRightButtonPressed = isRightButtonPressed;
    drivebase = Drivebase.getInstance();
  }

  @Override
  public void initialize() {
    System.out.println("Reset Gyros Command Running");
    if (isLeftButtonPressed.get() && isRightButtonPressed.get()) {
      gyroSystem.resetGyros();
      System.out.println("If block running reset gyros");
    }
  }
}
