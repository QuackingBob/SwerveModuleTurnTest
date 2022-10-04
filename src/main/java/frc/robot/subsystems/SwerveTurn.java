// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TalonFactory;

public class SwerveTurn extends SubsystemBase {
  private final BaseTalon[] modules;

  /** Creates a new SwerveTurn. */
  public SwerveTurn() {
    modules = new BaseTalon[4];
    modules[0] = TalonFactory.createTalonFX(2, false); // top left
    modules[1] = TalonFactory.createTalonFX(4, false); // back left
    modules[2] = TalonFactory.createTalonFX(6, false); // back right
    modules[3] = TalonFactory.createTalonFX(8, false); // top right
  }

  public void resetPID(double kP, double kI, double kD, double kF) {
    for (int i = 0; i < modules.length; i++) {
      modules[i] = TalonFactory.createTalonFX(2*i, false);
      modules[i].config_kP(Constants.Talon.kPIDIdx, kP);
      modules[i].config_kI(Constants.Talon.kPIDIdx, kI);
      modules[i].config_kD(Constants.Talon.kPIDIdx, kD);
      modules[i].config_kF(Constants.Talon.kPIDIdx, kF);
      modules[i].setSelectedSensorPosition(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
  }
}
