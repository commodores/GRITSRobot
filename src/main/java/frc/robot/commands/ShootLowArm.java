// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ShootLowArm extends CommandBase {
  /** Creates a new ShootLow. */
  private final Arm m_Arm;
  public ShootLowArm(Arm arm_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm_subsystem;
    addRequirements(m_Arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.setGoal(1.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
