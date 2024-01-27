package frc.robot.subsystems;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
public class ShooterSubsystem extends SubsystemBase{
    private final XboxController m_controller = new XboxController(0);
    Spark shooter = new Spark(2);
    public ShooterSubsystem() {}
    @Override
    public void periodic(){
        System.out.println("test");
        if(m_controller.getYButtonPressed()){
          shooter.set(0.5);
        }
        else if(m_controller.getAButtonPressed()){
          shooter.set(-0.5);
        }
        else{
        shooter.set(0);
        }
    }
}