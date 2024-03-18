// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedList;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants.GyroCrashDetection;

/** Gyro failure detection system. */
public class GyroSystem {

    private static GyroSystem gyroSystem;

    // The primary gyro.
    AHRS gyroMXP = new AHRS(Port.kMXP);

    // The backup gyro.
    AHRS gyroOnboard = new AHRS(Port.kOnboard);

    // The gyro to use. Setting this to the primary gyro to start.
    AHRS gyro = gyroMXP;

    boolean gyroMXPHasDied = false;
    boolean gyroOnboardHasDied = false;
    boolean hasBothDied = false;

    int gyroMXPTimer = 0;
    int gyroOnboardTimer = 0;

    // Used to keep a log of gyro measurement to check if there is noise in the
    // measurements (if there is noise, the gyro is alive).
    LinkedList<Double> gyroMXPMeasurements = new LinkedList<>();
    LinkedList<Double> gyroOnboardMeasurements = new LinkedList<>();

    int gyroMXPCount = 0;
    int gyroOnboardCount = 0;

    private GyroSystem() {
        resetGyros();
        SmartDashboard.putNumber("Reset the gyro system (1)", 0);
    }

    /**
     * Implements a rolling list of gyro measurments to check if the specified gyro
     * is dead. Gyro should have noise if it is still functional.
     */
    private boolean isGyroDead(LinkedList<Double> gyroMeasurements) {
        boolean isDead = true;

        if (gyroMeasurements.size() >= GyroCrashDetection.GYRO_MEASURMENTS_LIST_SIZE) {

            for (int i = 1; i < gyroMeasurements.size(); i++) {

                isDead = Math
                        .abs(gyroMeasurements.get(i) - gyroMeasurements
                                .get(i - 1)) < GyroCrashDetection.GYRO_NOISE_TOLERANCE
                        && isDead;
            }
        }

        return isDead;
    }

    /**
     * Remember to call this method every loop in periodic to update the list of
     * gyro measurments and figure out which gyros are still functional.
     */
    public void update() {
        boolean isEnabled = DriverStation.isEnabled();
        double hasDiedTimeLimit = GyroCrashDetection.HAS_DIED_TIME_LIMIT_IF_ENABLED;

        if (!isEnabled) {
            hasDiedTimeLimit = GyroCrashDetection.HAS_DIED_TIME_LIMIT_IF_DISABLED;
        }
        
        // Update the list of gyro measurments.
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

        // Update the gyro variable to the gyro that is still alive.
        if (!hasBothDied) {
            if (!isGyroDead(gyroMXPMeasurements) && !gyroMXPHasDied) {
                gyro = gyroMXP;
                gyroMXPTimer = 0;
            } else {

                gyroMXPTimer += 1;

                if (gyroMXPTimer > hasDiedTimeLimit) {
                    gyroMXPHasDied = true;
                }

                if (!isGyroDead(gyroOnboardMeasurements) && !gyroOnboardHasDied) {
                    gyro = gyroOnboard;
                    gyroOnboardTimer = 0;
                } else {
                    gyroOnboardTimer += 1;

                    if (gyroOnboardTimer > hasDiedTimeLimit) {
                        gyroOnboardHasDied = true;
                    }
                }
            }
        }

        if (gyroMXPHasDied && gyroOnboardHasDied) {
            hasBothDied = true;
        }

        SmartDashboard.putNumber("Gyro Wait Time", hasDiedTimeLimit);
        SmartDashboard.putNumber("Gyro MXP Timer", gyroMXPTimer);
        SmartDashboard.putNumber("Gyro Onboard Timer", gyroOnboardTimer);
        SmartDashboard.putNumber("Gyro Angle", getAngle());
        SmartDashboard.putNumber("Gyro MXP Angle", gyroMXP.getAngle());
        SmartDashboard.putNumber("Gyro Onboard Angle", gyroOnboard.getAngle());
        SmartDashboard.putBoolean("Gyro MXP Has Died", gyroMXPHasDied);
        SmartDashboard.putBoolean("Gyro Onboard Has Died", gyroOnboardHasDied);

        if (SmartDashboard.getNumber("Reset the gyro system (1)", 0) == 1) {
            resetGyros();
            SmartDashboard.putNumber("Reset the gyro system (1)", 0);
        }
    }

    public double getAngle() {

        if (!hasBothDied) {

            // Negative because the gyro yaw reads differently than wpilib.
            return -gyro.getAngle();
        } else {
            return 0.0;
        }
    }

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
        gyroMXPTimer = 0;
        gyroOnboardTimer = 0;
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
