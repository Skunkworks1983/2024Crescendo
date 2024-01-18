

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class Constants{



  // Motor, Encoder, & Joystick IDS
  public class IDS{

    // Drive Motor IDS
    public static final int RIGHT_FRONT_DRIVE = 7;
    public static final int RIGHT_BACK_DRIVE = 5;
    public static final int LEFT_FRONT_DRIVE = 3;
    public static final int LEFT_BACK_DRIVE = 1;

    // Turn Motor IDS
    public static final int RIGHT_FRONT_TURN = 11;
    public static final int RIGHT_BACK_TURN = 10;
    public static final int LEFT_FRONT_TURN = 9;
    public static final int LEFT_BACK_TURN = 12;

      // Turning Encoders
    public static final int RIGHT_FRONT_CAN_CODER = 8;
    public static final int RIGHT_BACK_CAN_CODER = 6;
    public static final int LEFT_FRONT_CAN_CODER = 4;
    public static final int LEFT_BACK_CAN_CODER = 2;

    // Joystick Ids
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
  }



  public class DrivebaseInfo{

    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double WHEEL_DIAMETER = .33333333;
    
    //public static final int TALON_TICKS_PER_MOTOR_REV = 2048;
    public static final double REVS_PER_FOOT = DRIVE_MOTOR_GEAR_RATIO / WHEEL_DIAMETER * Math.PI;

    // Module translations
    public static final double TRANSLATION_X = 1.895833333; // set this
    public static final double TRANSLATION_Y = 1.895833333; // set this

    // Can coder offsets
    public static final double FRONT_LEFT_OFFSET = 190.283203125/360.0;
    public static final double FRONT_RIGHT_OFFSET = 314.208984375/360.0;
    public static final double BACK_LEFT_OFFSET = 134.033203125/360.0;
    public static final double BACK_RIGHT_OFFSET = 13.623046875/360.0;
  }



  public class PIDControllers{

    public class TurnPID{

      // Turning Motor PID Constants
      public static final double KP = .005;
      public static final double KI = 0;
      public static final double KD = 0;//.003;
      public static final double PID_LOW_LIMIT = -.8;
      public static final double PID_HIGH_LIMIT = .8;
    }

    public class DrivePID{

      // Velocity Mode PID Constants
      public static final double KP = .1;
      public static final double KI = 0;
      public static final double KD = .02; // used to be 0
      public static final double KF = 0;
    }

    public class HeadingControlPID{

      public static final double KP = 0;
      public static final double KI = 0;
      public static final double KD = 0;

    }
  }

  

  // Speeds
  public static final double MAX_MODULE_SPEED = Units.feetToMeters(20); // used to be 10 - max speed that the module can go
  public static final double OI_DRIVE_SPEED_RATIO = 7.0; // max speed input is 15 fps in direction -- FAST
  public static final double OI_TURN_SPEED_RATIO = 360;  // max turn input in 360 degrees per second
  public static final double MAX_TRAJECTORY_SPEED = Units.feetToMeters(2.0); // max is 10 feet per second trajectory
  public static final double MAX_TRAJECTORY_ACCELERATION = Units.feetToMeters(30); // max acceleration is 10 fps squared

  // Field dimensions
  public static final double FIELD_X_LENGTH = 26.291667; // feet
  public static final double FIELD_Y_LENGTH = 54.2708333; // feet
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}