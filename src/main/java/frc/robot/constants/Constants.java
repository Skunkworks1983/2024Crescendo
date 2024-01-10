package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants{
    public static int frontLeftDriveMotorId = 3;
    public static int frontLeftTurnEncoderId = 4;
    public static int frontLeftTurnMotorId = 9;

    public static int frontRightDriveMotorId = 7;
    public static int frontRightTurnEncoderId = 8;
    public static int frontRightTurnMotorId = 11;

    public static int backLeftDriveMotorId = 1;
    public static int backLeftTurnEncoderId = 2;
    public static int backLeftTurnMotorId = 12;

    public static int ticksPerFoot= 13201;

    public static int backRightDriveMotorId = 5;
    public static int backRightTurnEncoderId = 6;
    public static int backRightTurnMotorId = 10;

    public final static Translation2d frontLeftLocation = new Translation2d(-0.381, 0.381);
    public final static Translation2d frontRightLocation = new Translation2d(0.381, 0.381);
    public final static Translation2d backLeftLocation = new Translation2d(-0.381, -0.381);
    public final static Translation2d backRightLocation = new Translation2d(0.381, -0.381);

}