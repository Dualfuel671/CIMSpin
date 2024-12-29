// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
//import photonLib

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;




import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
// xbox controller imports
import edu.wpi.first.wpilibj.Joystick;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  
  
  private Joystick xboxController = new Joystick(0);
  private TalonSRX motor = new TalonSRX(2);
  private PhotonCamera camera; //= new PhotonCamera("photonvision");

  
 


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
   

  }

  public void robotInit() {
    //camera.setDriverMode(true);
    //came  // Initialize the Talon SRX motor on CAN ID 1
      motor = new TalonSRX(2);
        
    // Initialize the Photon camera (make sure to use the correct camera name)
     camera = new PhotonCamera("Camera1"); // Replace with your camera namera.setLedMode(PhotonUtils.LedMode.kforceOn);

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
    boolean targetVisible = false; // Why does this say unused?
    SmartDashboard.putBoolean("Target Visible", targetVisible);
    
    double targetYaw = 0.0;
    double targetRange = 0.0;

    // Get the latest result from the camera
    var results = camera.getAllUnreadResults();

    // Check if there are any detected AprilTags
    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);

        // Check if there are any detected AprilTags
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 7) {
                    // Found Tag 7, record its information
                    targetYaw = target.getYaw();
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                        0.5, // Measured with a tape measure, or in CAD.
                        1.435, // From 2024 game manual for ID 7
                        Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                        Units.degreesToRadians(target.getPitch())
                    );
                    targetVisible = true;
                }
            }
        }
    }

    // Define a deadband
    double deadband = 5.0; // degrees

    // Determine target speed based on target's X position
    double targetSpeed = 0.0;
    double currentSpeed = 0.0;

    if (Math.abs(targetYaw) < deadband) {
        // Target is centered, stop the motor
        targetSpeed = 0.0;
    } else if (targetYaw > 0) {
        // Target is to the right, spin motor counterclockwise
        targetSpeed = -Constants.MAX_SPEED; // Adjust speed as necessary
    } else {
        // Target is to the left, spin motor clockwise
        targetSpeed = Constants.MAX_SPEED; // Adjust speed as necessary
    }

    // Smoothly transition to the target speed
    if (currentSpeed < targetSpeed) {
        currentSpeed += Constants.SMOOTHING_FACTOR; // Ramp up
    } else if (currentSpeed > targetSpeed) {
        currentSpeed -= Constants.SMOOTHING_FACTOR; // Ramp down
    }

    // Clamp to target speed
    if (currentSpeed > targetSpeed) {
        currentSpeed = targetSpeed;
    } else if (currentSpeed < targetSpeed) {
        currentSpeed = targetSpeed;
    }

    // Set the motor speed
    motor.set(ControlMode.PercentOutput, currentSpeed);
}


  
    






  @Override
  public void autonomousInit() {
    


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Get the Y-axis value from the left joystick
    double speed = xboxController.getRawAxis(0);
   
    // Create a motor controller
    TalonSRX motor = new TalonSRX(2); 

     // Set the motor controller to the Y-axis value
     motor.set(ControlMode.PercentOutput, speed);
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}



