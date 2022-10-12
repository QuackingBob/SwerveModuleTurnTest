// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class TurnMotor {
        public static final double gearRatio = 1.0/16.0/1.3;
        public static final double radius = 0.0508; // meters
        public static final double kwheelCircumference = 2*Math.PI*radius; // meters
    }

    public static class Talon {
        public static final int talonFXTicks = 2048;
        public static final int talonSRXTicks = 4096;

        public static final double MAX_VOLTAGE = 10.0;
    
        public static final int kPIDIdx = 0;
        public static final int kTimeoutMs = 10;
        public static final boolean kIsPracticeBot = false;
        public static final double kVoltageComp = 10.0;
        public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);
    }
}
