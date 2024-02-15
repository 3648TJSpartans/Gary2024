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
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import java.util.List;
import java.lang.Math;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  PhotonCamera camera = new PhotonCamera("Camera_Module_v1");
  double x = 0.3;
  boolean driveType = false;
  private Command m_autonomousCommand;
  Spark rightSpark = new Spark(1);
  Spark leftSpark = new Spark(0);
  Spark aim = new Spark(3);
  Spark shoot = new Spark(6);
  Servo servo = new Servo(4);
  //PWM redLed = new PWM(7);
  //PWM greenLed = new PWM(8);
  PWM blueLed = new PWM(9);
  private RobotContainer m_robotContainer;
  Joystick stick = new Joystick(0);
  private Timer time = new Timer();
  final double cameraHeight = Units.inchesToMeters(22);
  double targetHeight = Units.inchesToMeters(22);
  final double cameraPitchRad = Units.degreesToRadians(30);
  double goalDistance = Units.feetToMeters(0.5);
  PIDController forwardController = new PIDController(0.1, 0, 0);
  PIDController turnController = new PIDController(0.1, 0, 0);
  final double cameraAngle = 25;
  AddressableLED m_led = new AddressableLED(7);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(6);
  //DigitalOutput give = new DigitalOutput(1);
  DigitalInput giveIn = new DigitalInput(1);
  //DigitalOutput take = new DigitalOutput(2);



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    //give.set(true);
    //take.set(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    setColor(0, m_ledBuffer.getLength(), 0, 0, 0);
    x = 0.3;
    driveType = false;
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    setColor(0, m_ledBuffer.getLength(), 255, 255, 0);

    m_led.setData(m_ledBuffer);

    servo.setAngle(150);

    time.reset();
    time.start();

    rightSpark.set(0.3);
    leftSpark.set(-0.3);
    while (time.get() < 1) {;}
    for (int i = 0; i < 3; i++) {
      time.reset();
      time.start();
      rightSpark.set(0.3);
      leftSpark.set(0.3);
      while (time.get() < 1) {}
      rightSpark.set(0);
      leftSpark.set(0);
      shoot.set(0.5); 
      servo.setAngle(150);
      time.reset();
      time.start();
      while (time.get() < 0.095) {;}
      servo.setAngle(180);
      time.reset();
      time.start();
      while (time.get() < 1) {;}
      shoot.set(0.0);
    }

    setColor(0, m_ledBuffer.getLength(), 0, 0, 0);

    m_led.setData(m_ledBuffer);
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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
      /*var result = camera.getLatestResult();
      boolean hasTargets = result.hasTargets();
      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      int targetID = target.getFiducialId();
      */
      double forwardSpeed = 0;
      double rotationSpeed = 0;
      //Change driving type for Gary
      if (stick.getRawButtonPressed(3)) {
        if (driveType) {
          driveType = false;
          //setColor(0, m_ledBuffer.getLength(), 255, 0, 0);
        }
        else {
          driveType = true;
          //setColor(0, m_ledBuffer.getLength(), 0, 255, 0);
        }
      }
    //Increase and decrease speed of Gary
      if (stick.getRawButtonPressed(4)) {
        x += 0.1;
        if (x > 1) {
          x = 1;
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
        speedToPos(0.2, 10);
        if (result.hasTargets()) {
          if (result.getBestTarget().getFiducialId() == 2) {
            targetHeight = Units.feetToMeters(5);
            goalDistance = Units.feetToMeters(2.5);
            double range = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitchRad, Units.degreesToRadians(result.getBestTarget().getPitch()));
            forwardSpeed = -forwardController.calculate(range, goalDistance)*11;
            rotationSpeed = (-turnController.calculate(result.getBestTarget().getYaw(), 0));
            if (stick.getRawButtonPressed(3)) {
              System.out.println("Forward: " + forwardSpeed);
              System.out.println("Rotation: " + rotationSpeed);
              System.out.println("Range: " + range);
              System.out.println("CameraHeight: " + cameraHeight);
              System.out.println("TargetHeight: " + targetHeight);
              System.out.println("CameraPitchRad: " + cameraPitchRad);
              System.out.println("Last arg: " + Units.degreesToRadians(result.getBestTarget().getPitch()));
            }
          }
          double range = 
           PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitchRad, Units.degreesToRadians(result.getBestTarget().getPitch()));
          forwardSpeed = -forwardController.calculate(range, goalDistance)*70;
          rotationSpeed = (-turnController.calculate(result.getBestTarget().getYaw(), 0));
          
          
          

        } else {
          forwardSpeed = 0;
          rotationSpeed = 0;
        }
        leftSpark.set((-forwardSpeed + rotationSpeed)*0.2);
        rightSpark.set((forwardSpeed + rotationSpeed)*0.2);
      }
      else if (driveType) {
              leftSpark.set(stick.getRawAxis(1)*x);
              rightSpark.set(stick.getRawAxis(5)*-x);
      }
      else {
        leftSpark.set((stick.getRawAxis(1) + stick.getRawAxis(0))*x);
        rightSpark.set((-stick.getRawAxis(1) + stick.getRawAxis(0))*x);
      }

      //Aim the hopper
      if (!driveType) {
              aim.set(stick.getRawAxis(5)*-0.75);

      }
      //Shoot the ball
      if (stick.getRawAxis(3) > 0.2) {
        shoot.set(stick.getRawAxis(3));
      }
      else if (stick.getRawAxis(2) > 0.2) {
        shoot.set(-(stick.getRawAxis(2)));
      } else {
        shoot.set(0);
      }
      /*
      shoot.set(stick.getRawAxis(3)*1); 
      if (stick.getRawAxis(3) == 0) {
        shoot.set(stick.getRawAxis(2)*-1);
      }
      */
      //Open and close the servo
      if (stick.getRawButtonPressed(6)) {
        time.reset();
        time.start();
          servo.setAngle(100);
          while (time.get() < 0.5) {}
          servo.setAngle(180);
        }

      // if (stick.getRawButtonPressed(7)) {
      //   int m_rainbowFirstPixelHue = 1;
      //   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      //     int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      //     m_ledBuffer.setHSV(i, hue, 255, 128);
      //   leftSpark.set(0.4);
      //   rightSpark.set(0.4);
      //   time.reset();
      //   time.start();
      //   while (time.get() < 5) {
      //     if ((int) time.get() % 2 == 0) {
      //       shoot.set(0.5);
      //       servo.setAngle(100);
      //     } else {
      //       shoot.set(-0.5);
      //       servo.setAngle(180);
      //     }
      //     if (time.get() < 2.5) {
      //       aim.set(0.8);
      //     } else {
      //       aim.set(-0.8);
      //     }
      //   }
      //   time.reset();
      // }
      // if (stick.getRawButtonPressed(8)) {
      //   time.reset();
      //   time.start();
      //   while (time.get() < 5) {
      //     if ((int) time.get() % 2 == 0) {
      //       leftSpark.set(0.3);
      //       rightSpark.set(0.3);
      //       aim.set(0.3);
      //       servo.setAngle(180);
      //     } else {
      //       leftSpark.set(-0.3);
      //       leftSpark.set(-0.3);
      //       aim.set(-0.3);
      //       servo.setAngle(100);
      //     }
      //   }
      // }
      // if (stick.getPOV() == 0) {

      // }
      if (!giveIn.get()) {
        setColor(0, m_ledBuffer.getLength(), 200, 0, 0);
      } else {
        setColor(0, m_ledBuffer.getLength(), 0, 200, 0);
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
    setColor(0, m_ledBuffer.getLength(), 255, 164,0 );
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  public void speedToPos(double speed, double time) {
    this.time.reset();
    this.time.start();
    leftSpark.set(0);
    rightSpark.set(0);
    System.out.println("Method called");
    leftSpark.set(-speed);
    rightSpark.set(speed);
    while (this.time.get() < time) {
      
    }
    leftSpark.set(0);
    rightSpark.set(0);
  }

  // public double disPos() {
  //   var result = camera.getLatestResult();
  //   PhotonTrackedTarget tag = result.getBestTarget();
  //   double X = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitchRad, Units.degreesToRadians(result.getBestTarget().getPitch()));
  //   double N = X*Math.cos(25);
  //   return 
  // }

  //public double disToPos() {}
  public void getPose() {
    new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));

  }


  public void setColor(int start, int end, int r, int g, int b) {
    for (int i = start; i < end; i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

}




