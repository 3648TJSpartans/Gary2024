// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimeLightConstants;

import java.util.List;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Vision.TagVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  PhotonCamera camera = new PhotonCamera(LimeLightConstants.cameraName);
  double x = 0.3;
  boolean driveType = false;
  private Command m_autonomousCommand;
  Spark rightSpark = new Spark(1);
  Spark leftSpark = new Spark(0);
  Spark aim = new Spark(3);
  Spark shoot = new Spark(6);
  Servo servo = new Servo(4);
  private RobotContainer m_robotContainer;
  Joystick stick = new Joystick(1);
  private Timer time = new Timer();
  final double cameraHeight = Units.inchesToMeters(22);
  double targetHeight = Units.inchesToMeters(22);
  final double cameraPitchRad = Units.degreesToRadians(30);
  double goalDistance = Units.feetToMeters(3.5);
  PIDController forwardController = new PIDController(0.1, 0, 0);
  PIDController turnController = new PIDController(0.1, 0, 0);
  // TagVision vision = new TagVision();

  public void Robot() throws IOException {

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    servo.setAngle(150);

    time.reset();
    time.start();

    rightSpark.set(0.3);
    leftSpark.set(-0.3);
    while (time.get() < 1) {
      ;
    }
    for (int i = 0; i < 3; i++) {
      time.reset();
      time.start();
      rightSpark.set(0.3);
      leftSpark.set(0.3);
      while (time.get() < 1) {
        ;
      }
      rightSpark.set(0);
      leftSpark.set(0);
      shoot.set(0.5);
      servo.setAngle(150);
      time.reset();
      time.start();
      while (time.get() < 0.095) {
        ;
      }
      servo.setAngle(180);
      time.reset();
      time.start();
      while (time.get() < 1) {
        ;
      }
      shoot.set(0.0);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*
     * var result = camera.getLatestResult();
     * boolean hasTargets = result.hasTargets();
     * List<PhotonTrackedTarget> targets = result.getTargets();
     * PhotonTrackedTarget target = result.getBestTarget();
     * double yaw = target.getYaw();
     * double pitch = target.getPitch();
     * int targetID = target.getFiducialId();
     */
    double forwardSpeed;
    double rotationSpeed;
    // Change driving type for Gary
    if (stick.getRawButtonPressed(3)) {
      // vision.getEstimatedGlobalPose();
      if (driveType) {
        driveType = false;
      } else {
        driveType = true;
      }
    }
    // Increase and decrease speed of Gary
    if (stick.getRawButtonPressed(4)) {
      x += 0.1;
      if (x > 0.5) {
        x = 0.5;
      }
    }
    if (stick.getRawButtonPressed(1)) {
      x -= 0.1;
      if (x < 0.1) {
        x = 0.1;
      }
    }
    var result = camera.getLatestResult();
    if (stick.getRawButton(2)) {
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        if (target.getFiducialId() == 5) {
          targetHeight = Units.feetToMeters(5);
          goalDistance = Units.feetToMeters(3.5);
        }
        double range = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitchRad,
            Units.degreesToRadians(target.getPitch()));
        forwardSpeed = -forwardController.calculate(range, goalDistance) * 70;
        rotationSpeed = (-turnController.calculate(target.getYaw(), 0));
        if (stick.getRawButtonPressed(3)) {
          System.out.println("Forward: " + forwardSpeed);
          System.out.println("Rotation: " + rotationSpeed);
          System.out.println("Range: " + range);
          System.out.println("CameraHeight: " + cameraHeight);
          System.out.println("TargetHeight: " + targetHeight);
          System.out.println("CameraPitchRad: " + cameraPitchRad);
          System.out.println("Last arg: " + Units.degreesToRadians(result.getBestTarget().getPitch()));
        }

      } else {
        forwardSpeed = 0;
        rotationSpeed = 0;
      }
      leftSpark.set((forwardSpeed + rotationSpeed) * 0.2);
      rightSpark.set((-forwardSpeed + rotationSpeed) * 0.2);
    } else if (driveType) {
      leftSpark.set(stick.getRawAxis(1) * x);
      rightSpark.set(stick.getRawAxis(5) * -x);
    } else {
      leftSpark.set((stick.getRawAxis(1) + stick.getRawAxis(0)) * x);
      rightSpark.set((-stick.getRawAxis(1) + stick.getRawAxis(0)) * x);
    }

    // Aim the hopper
    if (!driveType) {
      aim.set(stick.getRawAxis(5) * -0.75);

    } else {
      aim.set(0);
    }
    // Shoot the ball
    if (stick.getRawAxis(3) > 0.4) {
      shoot.set(0.75);
    } else if (stick.getRawAxis(2) > 0.4) {
      shoot.set(-0.75);
    } else {
      shoot.set(0);
    }
    /*
     * shoot.set(stick.getRawAxis(3)*1);
     * if (stick.getRawAxis(3) == 0) {
     * shoot.set(stick.getRawAxis(2)*-1);
     * }
     */
    // Open and close the servo
    if (stick.getRawButtonPressed(6)) {
      time.reset();
      time.start();
      servo.setAngle(100);
      while (time.get() < 0.5) {
        ;
      }
      servo.setAngle(180);
    }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}