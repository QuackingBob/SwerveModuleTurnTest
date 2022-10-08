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
import java.util.Random;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TalonFactory;

public class AutoPID extends SubsystemBase {
  private BaseTalon motor;
  private TalonFXSimCollection motorSim;
  private Socket socket;
  private PrintWriter outputWriter;
  private BufferedReader inputStream;
  private double accumulatedSquareError = 0.0; // radian time hybrid squared
  private double[] pidf = {1,0,0,0};
  private double epsilon = 0.1; // radians
  private double maxTime = 1; // seconds
  private int M = 6; // number of points to test
  private int testsComplete = 0; // until this is N*M, keep going
  private double initialTime = 0.0; // used to calculate how long the test has run
  private double times; // the time to convergence for each module
  private double timeError; // this is used to compute delta times
  private double currError; // the current error of the module during the test
  private boolean moduleComplete = true; // which modules are ready for next point
  private double[] moduleSetPoints = new double[M]; // what is the current module setpoint
  private int moduleCurrentSetPointIdx = 0; // what is the current id pos that the modules are on
  private boolean passedThreshold = false; // whether the module has reached the desired point for the first time or not during the test
  private boolean runNextIteration = false; // whether to run the next batch of tests

  /** Creates a new SwerveTurn. */
  public AutoPID() {
    motor = TalonFactory.createTalonFX(2, false);
    motorSim = ((TalonFX)motor).getSimCollection();

    try {
      socket = new Socket("10.1.15.223", 5001);
    }
    catch (IOException e) {
      Logger.log(ErrorCode.InvalidHandle, "Could Not Open Socket");
      socket = new Socket();
    }
    for (int i = 0; i < M; i++) {
      double setPoint = Math.random() * 2 * Math.PI;
      moduleSetPoints[i] = setPoint;
    }
    startConnection();
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
    motor.config_kP(Constants.Talon.kPIDIdx, kP);
    motor.config_kI(Constants.Talon.kPIDIdx, kI);
    motor.config_kD(Constants.Talon.kPIDIdx, kD);
    motor.config_kF(Constants.Talon.kPIDIdx, kF);
    motor.setSelectedSensorPosition(0);
    motor.set(ControlMode.Velocity, 0);
    motorSim = ((TalonFX)motor).getSimCollection();
    motorSim.setIntegratedSensorRawPosition(0);
    motorSim.setIntegratedSensorRawPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    if (testsComplete < M && runNextIteration) {
      // if the module is not currently running a test
      if (moduleComplete) {
        times = Timer.getFPGATimestamp(); // set the time that it starts (times records the time it takes to first reach the position)
        initialTime = times; // this records the start time for reference of when to stop the test
        moduleComplete = false; // this means that the module is now running a test
        passedThreshold = false; // this means that it has not yet reached the desired position

        motor.set(ControlMode.Position, MathUtils.radiansToTicks(
          moduleSetPoints[moduleCurrentSetPointIdx], 
          Constants.Talon.talonFXTicks, 
          Constants.TurnMotor.gearRatio
        )); // this sets the motor to the test point

        motorSim.setIntegratedSensorRawPosition(
          MathUtils.radiansToTicks(
            moduleSetPoints[moduleCurrentSetPointIdx], 
            Constants.Talon.talonFXTicks, 
            Constants.TurnMotor.gearRatio
          )
        );
      }
      else {
        double currPos = MathUtils.ticksToRadians(
          motor.getSelectedSensorPosition(), 
          Constants.Talon.talonFXTicks, 
          Constants.TurnMotor.gearRatio); // this gets the current position of the robot once it is running the test

        if (MathUtils.withinEpsilon(
          currPos, 
          moduleSetPoints[moduleCurrentSetPointIdx], 
          epsilon
        )) {
          times = Timer.getFPGATimestamp() - times; // this sets times to the time it takes for the motor to first reach the desired position
          timeError = Timer.getFPGATimestamp(); // this sets timeError to the current time. This is used to find delta times to integrate error
          passedThreshold = true; // this indicates that the motor hass passed the desired position
        } // this checks that the motor is within a certain range of the desired position 

        if (passedThreshold) {
          double currTime = Timer.getFPGATimestamp(); // if it passed the desired position, get the current time
          currError += Math.abs(currPos - moduleSetPoints[moduleCurrentSetPointIdx]) * (currTime - timeError); // multiply the position error by delta time
          timeError = currTime; // set the timeError to current time to use again for dt
        } // do this only if it has already reached the desired point for the first time
      }

      if (Timer.getFPGATimestamp() - initialTime >= maxTime) { // check whether it has passed the maximum alloted time for the test to run
        moduleCurrentSetPointIdx++; // go to the next test position (index to module test position array)
        this.testsComplete++; // increment the number of tests completed
        accumulatedSquareError += moduleError(times, currError); // add the currentError of the module for this test to the accumulated square error accross all tests
        currError = 0; // set the current error to zero
        moduleComplete = true; // set the motor to be ready for the next test
        // motor.setSelectedSensorPosition(0);
        // motor.set(ControlMode.Position, 0);
        motor.set(ControlMode.Velocity, 0); // stop the motor
        if (testsComplete >= M) {
          runNextIteration = false; // if it is done with the maximum number of tests, stop running tests
        }
      }
    }
    else {
      sendData(); // if it's done with tests send the error data
      runNextIteration = false; // don't run more iterations yet
      moduleCurrentSetPointIdx = 0; // reset the setpoint index
    }

    sendData(); // send data again

    if (!runNextIteration) { // if it is not running an iteration and has sent data
      sendData(); // send data again
      this.pidf = recieveData(); // try to receive the next iteration of PIDF values
      resetPID(pidf[0], pidf[1], pidf[2], pidf[3]); // update the PID values
    }

    SmartDashboard.putNumber("Tests Complete", (double)testsComplete);
    SmartDashboard.putNumber("Accumulated Square error", accumulatedSquareError);
    SmartDashboard.putBoolean("Run Next Iteration", runNextIteration);
  }

  public void sendData() {
    SmartDashboard.putBoolean("Sent", true);
    outputWriter.println(netError()); // send the data via the output writer
  }

  public double[] recieveData() {
    try {
      String response = inputStream.readLine(); // get the line form the input stream
      double[] pidf = new double[4]; // get the new PIDF values
      String[] val = response.split(",");
      for (int i = 0; i < pidf.length; i++) {
        pidf[i] = Double.valueOf(val[i]);
      }
      this.runNextIteration = true; // if successful, then it should now run the next iteration
      this.testsComplete = 0; // set the tests completed to 0
      SmartDashboard.putString("PID Response", "" + pidf[0] + "," + pidf[1] + "," + pidf[2] + "," + pidf[3]);
      return pidf;
    }
    catch (IOException e) {
      Logger.log(ErrorCode.InvalidHandle, "Failed to Receive Data");
      return this.pidf;
    }
  }

  public double logistic(double x) { // used to contrain time error within certain bounds such that super high times are penalized about the same
    double L = 6;
    double k = 1.5;
    return L / (1 + Math.exp(-k * x)) - (L/2.0);
  }

  public double moduleError(double t, double e) {
    return Math.pow(logistic(t) + Math.abs(e), 2); // square the error so that high errors are penalized more
  }

  public double netError() {
    return this.accumulatedSquareError / (this.M - 1); // take the average of the square error (use sample average)
  }
}
