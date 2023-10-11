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
    private PathPlannerTrajectory trajectory;

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

        /////Auto Balance Tester//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> ScoreMidThanLow = PathPlanner.loadPathGroup("", new PathConstraints(1, 1));
        autos.put("ScoreMidThanLow", new SequentialCommandGroup(
            new Shoot(RobotContainer.m_Intake).withTimeout(2.5),
            new IntakeInn(RobotContainer.m_Intake).withTimeout(.5),
            getCommand(ScoreMidThanLow),
            new Shoot(RobotContainer.m_Intake).withTimeout(3)
        )); 
       
       
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        

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