// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class MoveSubsystem extends SubsystemBase {

  Spark rightSpark = new Spark(1);
  Spark leftSpark = new Spark(0);
  
  Spark aim = new Spark(3);
  Servo servo = new Servo(7);
  private final XboxController m_controller = new XboxController(0);
  private final  DifferentialDrive m_robotDrive = new DifferentialDrive(leftSpark, rightSpark);
  public MoveSubsystem() {}

  /** This function is called periodically during operator control. */
  @Override
  public void periodic() {
    //m_robotDrive.arcadeDrive(m_controller.getRightX(), m_controller.getRightX());
    rightSpark.set(m_controller.getRightX() * 0.5);
    leftSpark.set(m_controller.getLeftY() * -0.5);
    /* 
    shooter.set(joystick.getRawAxis(3) * -0.5);
    m_robotDrive.arcadeDrive(m_controller.getLeftY(), m_controller.getRightX());
    if(joystick.getPOV() == 0){
      aim.set(0.5);
    }else if(joystick.getPOV() == 180){
      aim.set(-0.5);
    }else{
      aim.set(0);
    }
    if(joystick.getRawButton(1)){
      servo.setAngle(120);
    }else{
      servo.setAngle(180);
    }
    */
  }


}
