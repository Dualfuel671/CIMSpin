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
      motor = new TalonSRX(1);
        
    // Initialize the Photon camera (make sure to use the correct camera name)
     camera = new PhotonCamera("photonvision"); // Replace with your camera namera.setLedMode(PhotonUtils.LedMode.kforceOn);

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
    // Get the latest result from the camera
   var result = camera.getAllUnreadResults();

    // Check if there are any detected AprilTags
    if (result.) {
        // Get the first detected target (AprilTag)
        var target = result.getBestTarget();

        // Get the target's X position (horizontal offset)
        double targetX = target.getYaw(); // This gives you the angle in degrees

        // Determine motor speed based on target's X position
        if (targetX > 0) {
            // Target is to the right, spin motor counterclockwise
            motor.set(ControlMode.PercentOutput, -0.5); // Adjust speed as necessary
        } else if (targetX < 0) {
            // Target is to the left, spin motor clockwise
            motor.set(ControlMode.PercentOutput, 0.5); // Adjust speed as necessary
        } else {
            // Target is centered, stop the motor
            motor.set(ControlMode.PercentOutput, 0);
        }

        // Optionally, display the target information on the SmartDashboard
        SmartDashboard.putNumber("Target X", targetX);
    } else {
        // No targets detected, stop the motor
        motor.set(ControlMode.PercentOutput, 0);
    }
}



//Testing still

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



