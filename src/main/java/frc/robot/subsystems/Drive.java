/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.CurvatureDrive;
import frc.robot.models.PairedTalonSRX;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final int LEFT_FRONT = 3,
                    LEFT_BACK = 1,
                    RIGHT_FRONT = 2,
                    RIGHT_BACK = 4,
                    PIGEON = 5;

  PairedTalonSRX leftPair, rightPair;
  PigeonIMU mPigeon;

  private final int PID_X = 0,
                    TIMEOUT_MS = 0;

  public final static double TICKS_PER_REV = 1024,
                              WHEEL_DIAMETER = 0.33; //feet

  public Drive(){
    leftPair = new PairedTalonSRX(LEFT_FRONT, LEFT_BACK);
    rightPair = new PairedTalonSRX(RIGHT_FRONT, RIGHT_BACK);
    mPigeon = new PigeonIMU(PIGEON);

    leftPair.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_X, TIMEOUT_MS);
    rightPair.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_X, TIMEOUT_MS);

    leftPair.setSelectedSensorPosition(0);
    rightPair.setSelectedSensorPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CurvatureDrive());
  }

  public void setCoast(){
    leftPair.setNeutralMode(NeutralMode.Coast);
    rightPair.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrake(){
    leftPair.setNeutralMode(NeutralMode.Brake);
    rightPair.setNeutralMode(NeutralMode.Brake);
  }

  public void resetEncoders(){
    leftPair.setSelectedSensorPosition(0, PID_X, TIMEOUT_MS);
    rightPair.setSelectedSensorPosition(0, PID_X, TIMEOUT_MS);
  }

  public void drive(ControlMode mode, double left, double right){
    leftPair.set(mode, left);
    rightPair.set(mode, right);
  }

  public void drive(double speed, double rotateValue){
    double leftMotorSpeed, rightMotorSpeed;
        if (speed > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = speed - rotateValue;
                rightMotorSpeed = Math.max(speed, rotateValue);
            } else {
                leftMotorSpeed = Math.max(speed, -rotateValue);
                rightMotorSpeed = speed + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-speed, rotateValue);
                rightMotorSpeed = speed + rotateValue;
            } else {
                leftMotorSpeed = speed - rotateValue;
                rightMotorSpeed = -Math.max(-speed, -rotateValue);
            }
        }
        drive(ControlMode.PercentOutput, leftMotorSpeed, rightMotorSpeed);
    }

  public int getLeftPosition(){
    return leftPair.getSelectedSensorPosition(PID_X);
  }

  public int getRightPosition(){
    return rightPair.getSelectedSensorPosition(PID_X);
  }

  public double getYaw(){
    double [] ypr = new double[3];
    mPigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public PigeonIMU getPigeon(){
    return mPigeon;
  }


  }

