// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class Robot extends TimedRobot {
  //private CANSparkMax m_rightLeadMotor, m_rightFollowMotor, m_leftLeadMotor, m_leftFollowMotor ;
  private CANSparkMax m_leftLeadMotor = new CANSparkMax(24, MotorType.kBrushless);
  private CANSparkMax m_leftFollowMotor = new CANSparkMax(23, MotorType.kBrushless);       
  private CANSparkMax m_rightLeadMotor = new CANSparkMax(22, MotorType.kBrushless);
  private CANSparkMax m_rightFollowMotor = new CANSparkMax(21, MotorType.kBrushless);
  private final Timer m_Timer = new Timer ();
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftLeadMotor, m_rightLeadMotor);
  private Joystick m_joystick;
  private final Relay m_relay = new Relay(0);
  private static final int kRelayForwardButton = 7;     // sets the button to button 7 to display a cone
  private static final int kRelayReverseButton = 8; // sets the button to button 8 to display a cube
  private static double triggerSpeed;
  private static double turnSpeed;
  private static double throttle;
  // This starts the IMU (Inertial Measurment Unit)
  public static final ADIS16448_IMU imu = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kX, SPI.Port.kMXP,ADIS16448_IMU.CalibrationTime._1s);
  public static double gyroYaw;
   //bellow is the code for Double Solenoid
  private final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 2, 3);
      private final DoubleSolenoid m_ExtenderDoubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6,7);
  private static final int kDoubleSolenoidForward = 3; //this maps the joystick button to be button 3
  private static final int kDoubleSolenoidReverse = 4; //this maps the joystick button to be button 4
  
  @Override
  public void robotInit() {

    
    imu.calibrate();
    m_leftLeadMotor.restoreFactoryDefaults();
    m_leftFollowMotor.restoreFactoryDefaults(); 
    m_rightLeadMotor.restoreFactoryDefaults();
    m_rightFollowMotor.restoreFactoryDefaults();
    m_rightLeadMotor.setInverted(true);
    m_leftLeadMotor.setIdleMode(IdleMode.kCoast);
    m_leftFollowMotor.setIdleMode(IdleMode.kCoast);
    m_rightLeadMotor.setIdleMode(IdleMode.kCoast);
    m_rightFollowMotor.setIdleMode(IdleMode.kCoast);
    m_rightFollowMotor.follow(m_rightLeadMotor);
    m_leftFollowMotor.follow(m_leftLeadMotor); 
    m_joystick = new Joystick(0);
      
  }

  @Override
  public void robotPeriodic() {
    gyroYaw = imu.getGyroAngleX();
    gyroYaw = gyroYaw % 360;
    SmartDashboard.putNumber("GyroAngle -Degrees", gyroYaw); //appears in shuffleboard
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Turn_Speed", -m_joystick.getX());
    turnSpeed = -m_joystick.getX()*throttle;
    double tx = LimelightHelpers.getTX("");
    SmartDashboard.putNumber("targetx", tx);
  }

  @Override
  public void autonomousInit() {
    m_Timer.reset();
    m_Timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    // drive for 2 seconds
    //code for lift if wanted
    //if (m_Timer.get() < 20){
      //m_doubleSolenoid.set(DoubleSolenoid.Value.kForward); //lifts up
    //} else if(m_Timer.get() < 22){
      //m_robotDrive.arcadeDrive(0.5, 0.0); //drives forward if less than 2 seconds
    //} else if(m_Timer.get() < 24){
      //m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    //} else if(m_Timer.get() < 26){
      //m_robotDrive.arcadeDrive(-0.5, 0.0);    
    //} else {
    //m_robotDrive.stopMotor();
    if (m_Timer.get()< 1.5){ m_robotDrive.stopMotor();
    } else if (m_Timer.get()< 2.5){
      m_robotDrive.arcadeDrive(0.3, 0.0); //drives forward if less than 1+ seconds
    } else if (m_Timer.get()< 3.5){
      m_robotDrive.arcadeDrive(-0.3, 0.0); //drives backwards if less than 1+ seconds
    } else if (m_Timer.get()< 7.0){
      m_robotDrive.arcadeDrive(0.0, (180 - gyroYaw) * 0.0035 + Math.signum(180 - gyroYaw)* 0.265); //rotates if less than 3+ seconds
   // } else if (m_Timer.get()< 4.0){ m_robotDrive.stopMotor();
    }else if (m_Timer.get()< 10.0){
        m_robotDrive.arcadeDrive(0.1, 0.0); //drives backward if less than 7.0+ seconds
   // }else if (m_Timer.get()< 12.0){
       // m_PusherdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }else if (m_Timer.get()< 14.9){
          m_robotDrive.arcadeDrive(0.0, 0.0); //stops if less than 14.9+ seconds
    }else {
      m_robotDrive.stopMotor();
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    boolean cone = m_joystick.getRawButton(kRelayForwardButton);
    boolean cube = m_joystick.getRawButton(kRelayReverseButton); 
    boolean LightsOff = m_joystick.getRawButton(9); // turns all lights off using button 9

    /*
     * Depending on the button values, we want to use one of
     * kOn, kOff, kForward, or kReverse. kOn sets both outputs to 12V,
     * kOff sets both to 0V, kForward sets forward to 12V
     * and reverse to 0V, and kReverse sets reverse to 12V and forward to 0V.
     */

    //cone is for if white or forward - so information can be communicated to user
    //cube is for if red or reverse - so information can be communicated to user
    
    if (cone) {
      m_relay.set(Relay.Value.kForward);
    } else if (cube) {
      m_relay.set(Relay.Value.kReverse);
    } else if (LightsOff) {
      m_relay.set(Relay.Value.kOff);
    } else {
     // m_relay.set(Relay.Value.kOff);
    }
    throttle = 1.0-(m_joystick.getThrottle()+1)/2;
    triggerSpeed = -m_joystick.getY()*throttle;
    turnSpeed = -m_joystick.getX()*throttle;


    m_robotDrive.arcadeDrive(triggerSpeed, turnSpeed);
    //m_robotDrive.arcadeDrive(-m_joystick.getY(), -m_joystick.getX());
  
    if (m_joystick.getRawButton(kDoubleSolenoidForward)) {// when using button 3 it makes the pnumatic lift up
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (m_joystick.getRawButton(kDoubleSolenoidReverse)) {// using button 4 makes the pnumatic go down
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);}
    
      else if (m_joystick.getRawButton(5)) {
      m_ExtenderDoubleSolenoid.set(DoubleSolenoid.Value.kForward); //if button 5 is pressed, scoop goes out
    } else if (m_joystick.getRawButton(6)) {
      m_ExtenderDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);  //if button 6 is pressed, scoop goes in
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
