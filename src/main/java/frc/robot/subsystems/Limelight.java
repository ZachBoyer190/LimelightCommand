/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry ledMode = table.getEntry("ledMode");
  NetworkTableEntry pipeline = table.getEntry("pipeline");
  
  boolean hasValidTarget = false,
          atTarget = false,
          blinkComplete = false,
          offComplete = false,
          onComplete = false;

  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean getAtTarget(){
    return atTarget;
  }

  public boolean getBlinkComplete(){
    return blinkComplete;
  }

  public void limelightBlink(){
    System.out.println("blink");
    ledMode.setNumber(2);
    blinkComplete = true;
  }

  public boolean limelightOff(){
    System.out.println("Off");
    ledMode.setNumber(1);
    offComplete = true;
    return offComplete;
  }
  
  public boolean limelightOn(){
    System.out.println("Off");
    ledMode.setNumber(3);
    onComplete = true;
    return onComplete;
  }

  public void limelightDefault(){
    pipeline.setNumber(0);
    ledMode.setNumber(0);
  }

  public void updateLimelight(){

    limelightDefault();

    //double tv = table.getEntry("tv").getDouble(0);
    double tx = table.getEntry("tx").getDouble(0);
    double ty = table.getEntry("ty").getDouble(0);
    //double ta = table.getEntry("ta").getDouble(0);

    hasValidTarget = true;

    double steer_cmd = 0.0,
           drive_cmd = 0.0, 
           heading_error = -tx, 
           distance_error = -ty, 
           min_command = 0.0,
           txDeadband = 1.0,
           kSteer = RobotMap.kSteerLimelight,
           kDrive = RobotMap.kDriveLimelight;

    if(tx > txDeadband){
      steer_cmd = kSteer*heading_error - min_command;
    } else if (tx < 1.0){
      steer_cmd = kSteer*heading_error + min_command;
    }

    drive_cmd = kDrive * distance_error;

    if(hasValidTarget){
      if(heading_error <= 4.0 && distance_error <= 1.0){
        atTarget = true;
        return;
      } else {
        System.out.println(tx + "     " + ty);
        atTarget = false;
        Robot.mDrive.drive(drive_cmd, steer_cmd);
      }
    } else {
      System.out.println("No Target");
      return;
    }
  }
}
