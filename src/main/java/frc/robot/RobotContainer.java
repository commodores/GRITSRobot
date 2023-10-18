package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<SequentialCommandGroup> autoChooser;
    private final AutoCommands autos;    
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Intake buttons */
    private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kB.value);

    //* Command Buttons */
    private final JoystickButton shootLow = new JoystickButton(driver, XboxController.Button.kX.value);


    /* Arm buttons */
    private final JoystickButton armDown = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armUp = new JoystickButton(driver, XboxController.Button.kRightBumper.value);


    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Intake m_Intake = new Intake();
    public static final Arm m_Arm = new Arm();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );

        autos = new AutoCommands(s_Swerve);
        autoChooser = new SendableChooser<>();
    
        Set<String> keys = autos.autos.keySet();
        autoChooser.setDefaultOption((String) keys.toArray()[1], autos.autos.get(keys.toArray()[1]));
        //keys.remove((String) keys.toArray()[0]);
        
        for (String i : autos.autos.keySet()) {
            autoChooser.addOption(i, autos.autos.get(i));
        }

        SmartDashboard.putData("Auto Selector", autoChooser);
        

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        /* Intake Commands */
        intakeIn.onTrue(new CubePickup(m_Arm,m_Intake));
        intakeIn.onFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(0, 0)));
      
        intakeOut.onTrue(new InstantCommand(() -> m_Intake.runIntakeSpeed(1, 1)));
        intakeOut.onFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(0, 0)));
        
        /*Arm Commands */
        armDown.onTrue(m_Arm.setArmGoalCommand(0.7));
        armUp.onTrue(m_Arm.setArmGoalCommand(3.05));

        /* Complex Commands */
        shootLow.onTrue(new ShootLow(m_Arm,m_Intake));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
     }
}
