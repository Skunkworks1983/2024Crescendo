// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedList;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants.GyroCrashDetection;

/**
 * System with gyro failure detection. Remember to call update() every loop in
 * periodic.
 */
public class GyroSystem {

    private static GyroSystem gyroSystem;

    // The primary gyro.
    AHRS gyroMXP = new AHRS(Port.kMXP);

    // The backup gyro.
    AHRS gyroOnboard = new AHRS(Port.kOnboard);

    // The gyro to use. Setting this to the primary gyro.
    AHRS gyro = gyroMXP;

    boolean gyroMXPHasDied = false;
    boolean gyroOnboardHasDied = false;
    boolean hasBothDied = false;

    int gyroMXPTimer = 0;
    int gyroOnboardTimer = 0;

    // Used to keep a log of gyro measurement to check if there is noise in the
    // measurements (the gyro is alive).
    LinkedList<Double> gyroMXPMeasurements = new LinkedList<>();
    LinkedList<Double> gyroOnboardMeasurements = new LinkedList<>();

    int gyroMXPCount = 0;
    int gyroOnboardCount = 0;

    private GyroSystem() {
        resetGyros();
        SmartDashboard.putNumber("Type in 1 to reset the gyros", 0.0);
    }

    /**
     * Implements a rolling list of gyro measurments to check if the specified gyro
     * is dead. Gyro
     * should have noise if it is still functional. Returns true if no gyro with the
     * name is
     * specified. Note
     */
    private boolean isGyroDead(AHRS gyroToCheck) {
        boolean isDead = true;

        if (gyroToCheck == gyroMXP) {

            if (gyroMXPMeasurements.size() >= GyroCrashDetection.GYRO_MEASURMENTS_LIST_SIZE) {

                // Iterate through each element in the list of gyro measurments. Setting i to 1
                // to
                // prevent
                // indexOutOfBounds.
                for (int i = 1; i < gyroMXPMeasurements.size(); i++) {

                    isDead = Math
                            .abs(gyroMXPMeasurements.get(i) - gyroMXPMeasurements
                                    .get(i - 1)) < GyroCrashDetection.GYRO_NOISE_TOLERANCE
                            && isDead;
                }
            }

        } else if (gyroToCheck == gyroOnboard) {

            if (gyroOnboardMeasurements.size() >= GyroCrashDetection.GYRO_MEASURMENTS_LIST_SIZE) {

                for (int i = 1; i < gyroOnboardMeasurements.size(); i++) {
                    isDead = Math
                            .abs(gyroOnboardMeasurements.get(i) - gyroOnboardMeasurements
                                    .get(i - 1)) < GyroCrashDetection.GYRO_NOISE_TOLERANCE
                            && isDead;
                }
            }
        }

        return isDead;
    }

    /**
     * Remember to call this method every loop in periodic; if you don't, the gyro
     * angle won't update.
     */
    public void update() {

        // Iterating by a step number (less measurments, reduces processing time)
        if (gyroMXPCount % GyroCrashDetection.COUNT_STEP_NUMBER == 0) {
            gyroMXPMeasurements.add(gyroMXP.getAngle());

            if (gyroMXPMeasurements
                    .size() > GyroCrashDetection.GYRO_MEASURMENTS_LIST_SIZE) {
                gyroMXPMeasurements.remove(0);
            }
        }

        gyroMXPCount++;

        // Same thing for the backup gyro
        if (gyroOnboardCount % GyroCrashDetection.COUNT_STEP_NUMBER == 0) {
            gyroOnboardMeasurements.add(gyroOnboard.getAngle());

            if (gyroOnboardMeasurements
                    .size() > GyroCrashDetection.GYRO_MEASURMENTS_LIST_SIZE) {
                gyroOnboardMeasurements.remove(0);
            }
        }

        gyroOnboardCount++;

        if (!hasBothDied) {
            if (!isGyroDead(gyroMXP) && !gyroMXPHasDied) {
                gyro = gyroMXP;
                gyroMXPTimer = 0;
            } else {

                gyroMXPTimer += 1;

                if (gyroMXPTimer > GyroCrashDetection.ISDEAD_TIME_LIMIT) {
                    gyroMXPHasDied = true;
                }

                if (!isGyroDead(gyroOnboard) && !gyroOnboardHasDied) {
                    gyro = gyroOnboard;
                    gyroOnboardTimer = 0;
                } else {
                    gyroOnboardTimer += 1;

                    if (gyroOnboardTimer > GyroCrashDetection.ISDEAD_TIME_LIMIT) {
                        gyroOnboardHasDied = true;
                    }
                }
            }
        }

        if (gyroMXPHasDied && gyroOnboardHasDied) {
            hasBothDied = true;
        }

        SmartDashboard.putNumber("Gyro Angle", getAngle());
        SmartDashboard.putNumber("Gyro MXP Angle", gyroMXP.getAngle());
        SmartDashboard.putNumber("Gyro Onboard Angle", gyroOnboard.getAngle());
        SmartDashboard.putBoolean("Gyro MXP Has Died", gyroMXPHasDied);
        SmartDashboard.putBoolean("Gyro Onboard Has Died", gyroOnboardHasDied);

        if (SmartDashboard.getNumber("Type in 1 to reset the gyros", 0) == 1) {
            resetGyros();
            SmartDashboard.putNumber("Type in 1 to reset the gyros", 0.0);
        }
    }

    /** Get the yaw angle reported by the gyro */
    public double getAngle() {

        if (!hasBothDied) {
            // Negative because the gyro reads differently than wpilib.
            return -gyro.getAngle();
        } else {
            return 0.0;
        }
    }

    /** Get the roll reported by the gyro (side to side rotation). */
    public double getRoll() {

        if (!hasBothDied) {
            return gyro.getRoll();
        } else {
            return 0.0;
        }
    }

    public double getPitch() {

        if (!hasBothDied) {
            return gyro.getPitch();
        } else {
            return 0.0;
        }
    }

    /** Reset the gyros and zero the yaw. */
    public void resetGyros() {
        gyroMXP.reset();
        gyroOnboard.reset();
        gyroMXP.zeroYaw();
        gyroOnboard.zeroYaw();
        gyroMXPHasDied = false;
        gyroOnboardHasDied = false;
        hasBothDied = false;
        System.out.println("Gyro system has been reset");
    }

    public boolean areBothGyrosDead() {
        return hasBothDied;
    }

    /** This method should only be used in drivebase in getGyroSystem method. */
    public static GyroSystem getInstance() {
        if (gyroSystem == null) {
            gyroSystem = new GyroSystem();
        }
        return gyroSystem;
    }
}
