// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GyroSystem;

/**
 * The reset gyro button must be pressed on both joysticks simaltenously in order to reset the
 * robots heading. This makes it more challenging to reset the robot's heading by accident.
 */
public class ResetGyros extends InstantCommand {

  GyroSystem gyroSystem;
  Supplier<Boolean> isLeftButtonPressed, isRightButtonPressed;

  public ResetGyros(Supplier<Boolean> isLeftButtonPressed, Supplier<Boolean> isRightButtonPressed) {
    this.isLeftButtonPressed = isLeftButtonPressed;
    this.isRightButtonPressed = isRightButtonPressed;
    gyroSystem = GyroSystem.getInstance();
  }

  @Override
  public void initialize() {
    System.out.println("Reset Gyros Command Running");
    if (isLeftButtonPressed.get() && isRightButtonPressed.get()) {
      gyroSystem.resetGyros();
    }
  }
}
