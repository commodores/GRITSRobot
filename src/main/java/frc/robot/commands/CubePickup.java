// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubePickup extends SequentialCommandGroup {
  public final Arm m_Arm;
  public final Intake m_Intake;
  /** Creates a new ShootLow. */
  public CubePickup(Arm ArmSubsystem, Intake IntakeSubsystem) {
    m_Arm = ArmSubsystem;
    m_Intake = IntakeSubsystem;
        
    addCommands(
      new IntakeInn(m_Intake),
      new ArmUp(m_Arm)
    );
  }
}
