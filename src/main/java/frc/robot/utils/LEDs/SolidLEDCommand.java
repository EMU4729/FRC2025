package frc.robot.utils.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class SolidLEDCommand extends LEDCommand {
  protected Color color;
  private Command superRef;

  public SolidLEDCommand(Color color) {
    this.color = color;
  }
  public SolidLEDCommand withRef(Command ref){
    superRef = ref;
    return this;
  }

  @Override
  public void initialize() {
    zones.forEach((zone)->zone.apply(color));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    zones.forEach((zone)->zone.apply(Color.kBlack));

    if(interrupted && superRef != null){superRef.cancel();}
  }

  @Override
  public boolean isFinished(){
    return false;
  } 
}
