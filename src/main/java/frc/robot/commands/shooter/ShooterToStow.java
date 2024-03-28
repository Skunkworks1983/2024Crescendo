// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.LimitSwitch;
import frc.robot.subsystems.SubsystemGroups;
import frc.robot.subsystems.SubsystemGroups.Subsystems;

public class ShooterToStow extends Command {

    private Shooter shooter;

    public ShooterToStow() {
        shooter = Shooter.getInstance();
        addRequirements(SubsystemGroups.getInstance(Subsystems.SHOOTER_PIVOT));
    }

    @Override
    public void initialize() {
        // this is the target position for the pidcontroller, which is the resting angle plus 5
        // degrees. Since we cut off 10 degrees before the resting position, we will go downward
        // fast with the shooter, then switch over to percent output for the last bit of the way

        shooter.setPivotMotorPercentOutput(Constants.Shooter.SHOOTER_PIVOT_FAST_SPEED);
        System.out.println("Shooter to Stow Command Initialize");
    }

    @Override
    public void execute() {

        if (shooter.getShooterPivotRotationInDegrees() <= Constants.Shooter.SHOOTER_RESTING_POSITION
                .getDegrees() + Constants.Shooter.PIVOT_STOW_OFFSET) {
            shooter.setPivotMotorPercentOutput(-Constants.Shooter.SHOOTER_PIVOT_SLOW_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setPivotMotorPercentOutput(0);
        shooter.setFlywheelSetpoint(Constants.Shooter.STOW_FLYWHEEL_SPEED);
        if(interrupted) {
            System.out.println("Shooter to Stow Command Interupted");
        } else {
            System.out.println("Shooter to Stow Command End");
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.getLimitSwitchOutput(LimitSwitch.REVERSE_LIMIT_SWITCH);
    }
}
