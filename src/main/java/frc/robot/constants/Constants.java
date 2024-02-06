

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

  // Motor, Encoder, & Joystick IDS
  public class IDS {

    // Drive Motor IDS
    public static final int RIGHT_FRONT_DRIVE = 14;
    public static final int RIGHT_BACK_DRIVE = 1;
    public static final int LEFT_FRONT_DRIVE = 15;
    public static final int LEFT_BACK_DRIVE = 0;

    // Turn Motor IDS
    public static final int RIGHT_FRONT_TURN = 10;
    public static final int RIGHT_BACK_TURN = 5;
    public static final int LEFT_FRONT_TURN = 11;
    public static final int LEFT_BACK_TURN = 4;

      // Turning Encoders
    public static final int RIGHT_FRONT_CAN_CODER = 17;
    public static final int RIGHT_BACK_CAN_CODER = 18;
    public static final int LEFT_FRONT_CAN_CODER = 16;
    public static final int LEFT_BACK_CAN_CODER = 19;

    // Joystick Ids
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int BUTTON_STICK = 2;
    public static final int TARGETING_BUTTION = 11;
  }



  public class DrivebaseInfo {

    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double WHEEL_DIAMETER = .33333333; //feet
    
    //public static final int TALON_TICKS_PER_MOTOR_REV = 2048;
    public static final double REVS_PER_FOOT = DRIVE_MOTOR_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);

    // Module translations
    public static final double TRANSLATION_X = 0.947916666; //feet
    public static final double TRANSLATION_Y = 0.822916666; //feet

    // Can coder offsets
    public static final double FRONT_LEFT_OFFSET = 0.262451; //rotations, not actual values, get them again
    public static final double FRONT_RIGHT_OFFSET = -0.229492; //rotations
    public static final double BACK_LEFT_OFFSET = -0.348145; //rotations
    public static final double BACK_RIGHT_OFFSET = 0.295654; //rotations
  }



  public class PIDControllers {

    public class TurnPID {

      // Turning Motor PID Constants
      public static final double KP = .002; //.005
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double PID_LOW_LIMIT = -.8;
      public static final double PID_HIGH_LIMIT = .8;
      public static final double TURN_PID_TOLERANCE = 2;
    }

    public class DrivePID {

      // Velocity Mode PID Constants
      public static final double KP = .25; //.25
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KF = 0;
    }

    public class HeadingControlPID {

      public static final double KP = 6.5;
      public static final double KI = 0;
      public static final double KD = 0;

    }
  }

  
  // Speed & Deadband
  public static final double X_JOY_DEADBAND = .1;
  public static final double Y_JOY_DEADBAND = .1;
  public static final double ROT_JOY_DEADBAND = .2;
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(20);
  public static final double OI_DRIVE_SPEED_RATIO = 7; // 7.0 is slow
  public static final double OI_TURN_SPEED_RATIO = 360;  // max turn speed is 360 degrees per second
  public static final double MAX_TRAJECTORY_SPEED = Units.feetToMeters(2.0);
  public static final double MAX_TRAJECTORY_ACCELERATION = Units.feetToMeters(30);
  public static final String CANIVORE_NAME = "1983 Comp Drivebase";
  public static final Pose2d START_POSITION = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
  public static final double TIME_UNTIL_HEADING_CONTROL = 1; // seconds

  // Field dimensions and positions
  public static final double FIELD_X_LENGTH = 26.291667; // feet
  public static final double FIELD_Y_LENGTH = 54.2708333; // feet
  public static final double TARGETING_POSITION_X = 0; //the position that the targeting buttion will point at
  public static final double TARGETING_POSITION_Y = 0;
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}