// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  // Controller
  XboxController C_XboxController = new XboxController(0);
  boolean Abutton = new XboxController(0).getAButton();

  // Left side drivetrain motor group
  // MAKE SURE INDEX IS CORRECT
  WPI_TalonSRX m_leftMotorB = new WPI_TalonSRX(0);
  WPI_TalonSRX m_leftMotorF = new WPI_TalonSRX(1);
  MotorControllerGroup m_Left = new MotorControllerGroup(m_leftMotorB, m_leftMotorF);
  // right side Drivetrain Motor Group
  // DOUBLE MAKE SURE INDEX IS CORRECT
  WPI_TalonSRX m_RightMotorB = new WPI_TalonSRX(0);
  WPI_TalonSRX m_RightMotorF = new WPI_TalonSRX(1);
  MotorControllerGroup m_Right = new MotorControllerGroup(m_RightMotorB, m_RightMotorF);
  // Drive train using both left and right motor controller groups
  DifferentialDrive m_Drive = new DifferentialDrive(m_Left, m_Right);

  //Sweeper Motor - FIND OUT WHAT MOTOR CONTROLLER IT USES AND DEVICE NUMBER
  Talon m_SweeperMotor = new Talon(0);
  //ElevatorMotor - FIND OUT WHAT MOTOR CONTROLLER IT USES AND DEVICE NUMBER
  Talon m_ElevatorMotor = new Talon(1);
  // Sweeper Motor Controller Group
  MotorControllerGroup SweeperGroup = new MotorControllerGroup(m_SweeperMotor, m_ElevatorMotor);

  //Shooter Motor
  Talon m_ShooterMotor = new Talon(2);
  //Shooter Motor Group
  MotorControllerGroup ShooterGroup = new MotorControllerGroup(m_ElevatorMotor, m_ShooterMotor);
  // Sweeps ball and puts it up elevator by activating "SweeperGroup" Motor Controller Group
  public void SweepBall(MotorControllerGroup SweeperGroup){
    SweeperGroup.set(1.0);
  }

  //shoots ball and reloads from elevator
  public void ShootBall(MotorControllerGroup ShooterGroup){
    double MotorSpeed = C_XboxController.getRightTriggerAxis();
    ShooterGroup.set(MotorSpeed);
  }



  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_Right.setInverted(true);

  }

  @Override
  public void teleopPeriodic() {
    m_Drive.tankDrive(C_XboxController.getLeftX(), C_XboxController.getRightX());
    // turns on sweeper and elevator motor if A button is pressed it is boolean so I assume when its not pressed the if statement will stop
    if (C_XboxController.getAButton()){
      SweepBall(SweeperGroup);
    }
    //Shoots ball by turning on elevator and shooter motors same as sweepr but different motors
    if (C_XboxController.getRightTriggerAxis() > 0.5){
      ShootBall(ShooterGroup);
    }


  }
}
