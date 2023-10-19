// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;;

public class AutoCommands {

    private final Swerve swerve;
    public final Map<String, SequentialCommandGroup> autos;
    public final Map<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;

    //Example Multi-Path
    

    public AutoCommands(Swerve swerve) {
        
        this.swerve = swerve;

        //Build Autos
        autos = new HashMap<String, SequentialCommandGroup>();
        eventMap = new HashMap<String, Command>();
        
        /////Do Nothing//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        autos.put("nothing", new SequentialCommandGroup(
          new Nothing()
        ));

        /////Score mid then shoot two in on non substation side//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> BumpThreePiece = PathPlanner.loadPathGroup("BumpThreePiece", new PathConstraints(2, 2));
        autos.put("BumpThreePiece", new SequentialCommandGroup(  
            new Shoot(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(BumpThreePiece),
            new Shoot(RobotContainer.m_Intake).withTimeout(1)
        )); 
       
       
        /////Score mid than low on substation side ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> NonBumpThreePiece = PathPlanner.loadPathGroup("NonBumpThreePiece", new PathConstraints(3.5, 3));
        autos.put("NonBumpThreePiece", new SequentialCommandGroup(  
            new Shoot(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(NonBumpThreePiece),
            new Shoot(RobotContainer.m_Intake).withTimeout(0.5)
        ));

        /////charge ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> Charge = PathPlanner.loadPathGroup("Charge", new PathConstraints(1.5, 1));
        autos.put("Charge", new SequentialCommandGroup(  
            new Shoot(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(Charge),
            new AutoBalanceCommand(RobotContainer.s_Swerve)
        ));

        //Events////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        eventMap.put("ArmDown", new ArmDown(RobotContainer.m_Arm));
        eventMap.put("IntakeInn", new IntakeInn(RobotContainer.m_Intake));
        eventMap.put("ArmUp", new ArmUp(RobotContainer.m_Arm));
        eventMap.put("ShootLow", new ShootLow(RobotContainer.m_Arm, RobotContainer.m_Intake));
        eventMap.put("CubePickup", new CubePickup(RobotContainer.m_Arm, RobotContainer.m_Intake));
        eventMap.put("Shoot", new Shoot(RobotContainer.m_Intake));
        eventMap.put("StopIntake", new StopIntake(RobotContainer.m_Intake));
        
        

    }

    private Command getCommand(List<PathPlannerTrajectory>pathGroup) {                
        
        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXandYControllers, 0, 0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve);
        return autoBuilder.fullAuto(pathGroup);
    }

    

}