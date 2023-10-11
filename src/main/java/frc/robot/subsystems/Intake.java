// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




public class Intake extends SubsystemBase {
  

  private final CANSparkMax intakeInnerMotor;
  private final CANSparkMax intakeOuterMotor;
  private final TimeOfFlight m_rangeSensor;
  

  /** Creates a new Intake. */
  public Intake() {

    intakeInnerMotor = new CANSparkMax(Constants.IntakeConstants.intakeInner, MotorType.kBrushless);
    intakeInnerMotor.restoreFactoryDefaults();
    intakeInnerMotor.setSmartCurrentLimit(30);
    intakeInnerMotor.setIdleMode(IdleMode.kBrake);

    intakeOuterMotor = new CANSparkMax(Constants.IntakeConstants.intakeOuter, MotorType.kBrushless);
    intakeOuterMotor.restoreFactoryDefaults();
    intakeOuterMotor.setSmartCurrentLimit(30);
    intakeOuterMotor.setIdleMode(IdleMode.kBrake);

    m_rangeSensor = new TimeOfFlight(22);
    m_rangeSensor.setRangingMode(RangingMode.Short, 40);
    
  }

  public void runIntakeSpeed(double speedInner, double speedOuter){
      intakeInnerMotor.set(speedInner);
      intakeOuterMotor.set(speedOuter);   
  }

  public double getDistance(){
    return m_rangeSensor.getRange();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", m_rangeSensor.getRange());
  }

}