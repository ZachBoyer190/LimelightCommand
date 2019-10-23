/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.CurvatureDrive;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX mLeft1 = new WPI_TalonSRX(RobotMap.leftDrive1);
  private WPI_TalonSRX mLeft2 = new WPI_TalonSRX(RobotMap.leftDrive2);
  private WPI_TalonSRX mRight1 = new WPI_TalonSRX(RobotMap.rightDrive1);
  private WPI_TalonSRX mRight2 = new WPI_TalonSRX(RobotMap.rightDrive2);


  private SpeedControllerGroup leftMotors = new SpeedControllerGroup(mLeft1, mLeft2);
  private SpeedControllerGroup rightMotors = new SpeedControllerGroup(mRight1, mRight2);
  private DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CurvatureDrive());
  }

  public void drive(double throttle, double steer, boolean quickTurn){
    m_drive.curvatureDrive(throttle, steer, quickTurn);
  }

  public void drive(double throttle, double steer){
    m_drive.arcadeDrive(throttle, steer);
  }
}
