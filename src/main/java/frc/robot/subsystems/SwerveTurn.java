// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.Console;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.*;
import java.sql.ClientInfoStatus;
import java.util.Random;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Talon;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TalonFactory;

public class SwerveTurn extends SubsystemBase {
  private final BaseTalon[] modules;
  private final Socket socket;
  private PrintWriter outputWriter;
  private BufferedReader inputStream;
  private double error = 0.0; // radian time hybrid
  private double accumulatedSquareError = 0.0;
  private double pidf[] = {1,0,0,0};
  private double epsilon = 0.1; // radians
  private double maxTime = 1; // seconds
  private int N = 4; // number of modules
  private int M = 6; // number of points to test
  private int testsComplete = 0; // until this is N*M, keep going
  private double times[] = new double[M]; // the time to convergence for each module
  private boolean[] moduleComplete = {true, true, true, true}; // which modules are ready for next point
  private double[] moduleSetPoints = new double[M]; // what is the current module setpoint
  private int[] moduleCurrentSetPointIdx = new int[M];

  /** Creates a new SwerveTurn. */
  public SwerveTurn() {
    modules = new BaseTalon[4];
    modules[0] = TalonFactory.createTalonFX(2, false); // top left
    modules[1] = TalonFactory.createTalonFX(4, false); // back left
    modules[2] = TalonFactory.createTalonFX(6, false); // back right
    modules[3] = TalonFactory.createTalonFX(8, false); // top right

    socket = new Socket();
    for (int i = 0; i < M; i++) {
      double setPoint = Math.random() * 2 * Math.PI;
      moduleSetPoints[i] = setPoint;
    }
  }

  public void startConnection() {
    try {
      outputWriter = new PrintWriter(socket.getOutputStream());
      inputStream = new BufferedReader(new InputStreamReader(socket.getInputStream()));
    }
    catch (IOException e) {
      Logger.log(ErrorCode.InvalidHandle, "Failed To Connect");
    }
  }

  public void resetPID(double kP, double kI, double kD, double kF) {
    for (int i = 0; i < modules.length; i++) {
      modules[i] = TalonFactory.createTalonFX(2*i, false);
      modules[i].config_kP(Constants.Talon.kPIDIdx, kP);
      modules[i].config_kI(Constants.Talon.kPIDIdx, kI);
      modules[i].config_kD(Constants.Talon.kPIDIdx, kD);
      modules[i].config_kF(Constants.Talon.kPIDIdx, kF);
      modules[i].setSelectedSensorPosition(0);
      modules[i].set(ControlMode.Velocity, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    if (testsComplete < M*N) {
      for (int i = 0; i < M; i++) {
        if (moduleComplete[i]) {
          times[i] = Timer.getFPGATimestamp();
          moduleComplete[i] = false;
          modules[i].set(ControlMode.Position, MathUtils.radiansToTicks(
            moduleSetPoints[moduleCurrentSetPointIdx[i]], 
            Constants.Talon.talonFXTicks, 
            Constants.TurnMotor.gearRatio
          ));
        }
        else {
          if (MathUtils.withinEpsilon(
            MathUtils.ticksToRadians(
              modules[i].getSelectedSensorPosition(), 
              Constants.Talon.talonFXTicks, 
              Constants.TurnMotor.gearRatio), 
            moduleSetPoints[moduleCurrentSetPointIdx[i]], epsilon
          )) {
            times[i] = Timer.getFPGATimestamp() - times[i];
            
          }
        }
      }
    }
  }

  public void sendData() {
    outputWriter.print(error + "\n");
  }

  public double[] recieveData() {
    try {
      String response = inputStream.readLine();
      double[] pidf = new double[4];
      String[] val = response.split(",");
      for (int i = 0; i < pidf.length; i++) {
        pidf[i] = Double.valueOf(val[i]);
      }
      return pidf;
    }
    catch (IOException e) {
      Logger.log(ErrorCode.InvalidHandle, "Failed to Receive Data");
      return this.pidf;
    }
  }

  public double logistic(double x) {
    double L = 6;
    double k = 1.5;
    return L / (1 + Math.exp(-k * x)) - (L/2.0);
  }

  public double moduleError(double t, double e) {
    return logistic(t) + Math.abs(e);
  }

  public double netError() {
    return accumulatedSquareError / (N*M);
  }
}
