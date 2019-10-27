/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.models.PathfinderSequence;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;

public class DriveSequence extends Command {

  Drive drive = Robot.mDrive;
  PathfinderSequence sequence;
  boolean resetSensors;
  private EncoderFollower leftFollower, rightFollower;
  
  public DriveSequence(PathfinderSequence sequence, boolean resetSensors) {
    requires(Robot.mDrive);
    this.sequence = sequence;
    this.resetSensors = resetSensors;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(resetSensors){
      Robot.mDrive.resetEncoders();
    }

    Robot.mDrive.setBrake();

    leftFollower = new EncoderFollower(sequence.getLeft());
    leftFollower.configureEncoder(Robot.mDrive.getLeftPosition(), (int)Robot.mDrive.TICKS_PER_REV, Robot.mDrive.WHEEL_DIAMETER);
    leftFollower.configurePIDVA(0.75, 0, 0, 1 / 12, 0);
    
    rightFollower = new EncoderFollower(sequence.getRight());
    rightFollower.configureEncoder(Robot.mDrive.getRightPosition(), (int)Robot.mDrive.TICKS_PER_REV, Robot.mDrive.WHEEL_DIAMETER);
    rightFollower.configurePIDVA(0.75, 0, 0, 1 / 12, 0);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.mDrive.getPigeon().getState() != PigeonState.Ready){
      double leftSpeed = leftFollower.calculate(Robot.mDrive.getLeftPosition());
      double rightSpeed = rightFollower.calculate(Robot.mDrive.getRightPosition());

      double gyroHeading = -Robot.mDrive.getYaw();
      double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
      double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
      double turn = angleDifference / 600; //Magic 254 number

      Robot.mDrive.drive(ControlMode.PercentOutput, leftSpeed + turn, rightSpeed - turn);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return leftFollower.isFinished() && rightFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.mDrive.drive(ControlMode.PercentOutput, 0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
