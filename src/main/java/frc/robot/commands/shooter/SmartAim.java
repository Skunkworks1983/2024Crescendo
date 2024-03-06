// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.ShooterAimUtils;

public class SmartAim extends Command {
  Shooter shooter;
  Drivebase drivebase;

  /** Creates a new SmartAim. */
  public SmartAim() {
    shooter = Shooter.getInstance();
    drivebase = Drivebase.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Translation3d pivotPositionFieldReletive =
        ShooterAimUtils.calculatePivotPositionFieldReletive(drivebase.getGyroAngle(),
            Units.radiansToDegrees(shooter.getShooterPivotRotationInDegrees()), drivebase.getRobotPose().getTranslation());

    double flywheelSpeed =
    Constants.Shooter.DEFUALT_SPEAKER_FLYWHEEL_SPEED;//ShooterAimUtils.calculateIdealFlywheelSpeed(pivotPositionFieldReletive.toTranslation2d());
    //shooter.setFlywheelSetpoint(flywheelSpeed);

    double shooterAngle = ShooterAimUtils
        .calculateIdealStationaryShooterPivotAngle(pivotPositionFieldReletive, flywheelSpeed);

    // 90 - theta is neccecary to convert from the system in which forward is 90 and up is 0 to the
    // system in which 0 is forward and 90 is upward.
    //TODO: enable temporarily disabled code after testing.
    SmartDashboard.putNumber("smart aim rotation set with normal rotationt", Units.radiansToDegrees(shooterAngle));
    SmartDashboard.putNumber("smart aim rotation set", new Rotation2d((Math.PI / 2.0) - shooterAngle).getDegrees());//shooter.setPivotAngleAndSpeed(new Rotation2d((Math.PI / 2) - shooterAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFlywheelSetpoint(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
