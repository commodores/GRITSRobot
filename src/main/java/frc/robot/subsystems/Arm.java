// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Arm extends TrapezoidProfileSubsystem {

  // Define the motor and encoders
  private final CANSparkMax armLeader;
  private final CANSparkMax armFollower;

  private final SparkMaxPIDController m_PIDController;
  private final RelativeEncoder m_relative_encoder;
  private final AbsoluteEncoder m_absolute_encoder;

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.ArmConstants.kSVolts, Constants.ArmConstants.kGVolts,
          Constants.ArmConstants.kVVoltSecondPerRad, Constants.ArmConstants.kAVoltSecondSquaredPerRad);

  /** Creates a new ArmShoulder. */
  public Arm() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
      2.96);

    armLeader = new CANSparkMax(Constants.ArmConstants.armLeader, MotorType.kBrushless);
    armFollower = new CANSparkMax(Constants.ArmConstants.armFollower, MotorType.kBrushless);

    armLeader.restoreFactoryDefaults();
    armLeader.setInverted(false);
    armLeader.setSmartCurrentLimit(30);
    armLeader.setIdleMode(IdleMode.kBrake);

    armFollower.restoreFactoryDefaults();
    armFollower.follow(armLeader, true);

    m_PIDController = armLeader.getPIDController();
    m_PIDController.setP(Constants.ArmConstants.kP);
    m_PIDController.setI(Constants.ArmConstants.kI);
    m_PIDController.setD(Constants.ArmConstants.kD);
    m_PIDController.setFF(Constants.ArmConstants.kFF);

    m_relative_encoder = armLeader.getEncoder();
    m_relative_encoder.setPositionConversionFactor((2 * Math.PI) / Constants.ArmConstants.kGearRatio); //Converted to Radians

    m_absolute_encoder = armLeader.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_absolute_encoder.setPositionConversionFactor(Math.PI * 2); //Converted to Radians?

    m_PIDController.setFeedbackDevice(m_absolute_encoder);

  }

  @Override
  public void periodic() {
    double relativeEncoderValue = m_relative_encoder.getPosition();
    double absoluteEncoderValue = m_absolute_encoder.getPosition();
    
    // Display current values on the SmartDashboard
    //SmartDashboard.putNumber("arm Output", armLeader.getAppliedOutput());
    SmartDashboard.putNumber("Shoulder Relative Encoder Degrees", Units.radiansToDegrees(relativeEncoderValue));
    SmartDashboard.putNumber("Shoulder Absolute Encoder Degrees", Units.radiansToDegrees(absoluteEncoderValue));
    SmartDashboard.putNumber("Shoulder Relative Encoder Radians", relativeEncoderValue);
    SmartDashboard.putNumber("Shoulder Absolute Encoder Radians", absoluteEncoderValue);

    // Execute the super class periodic method
    super.periodic();
  }

  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    // Calculate the feedforward fromteh setPoint
    double feedforward = m_feedforward.calculate(setPoint.position, setPoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    // The ArmFeedForward computes in radians. We need to convert back to degrees.
    // Remember that the encoder was already set to account for the gear ratios.
    
    m_PIDController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
    
    //SmartDashboard.putNumber("Shoulder Feedforward", feedforward);
    //SmartDashboard.putNumber("Shoulder SetPoint", Units.metersToInches(setPoint.position));
    //SmartDashboard.putNumber("Shoulder Velocity", Units.metersToInches(setPoint.velocity));
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

}