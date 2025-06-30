package frc.robot.utils.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RepeatedFlashLEDCommand extends Command{
  private SequentialCommandGroup flashSequence = new SequentialCommandGroup();
  private int loopIdx = 0;
  private int iterations = 0;

  public RepeatedFlashLEDCommand(FlashSolidLEDCommand flashCommand, int iterations){
    this.flashSequence.addCommands(flashCommand.withRef(this));
    this.flashSequence.addCommands(new FlashSolidLEDCommand(Color.kBlack, flashCommand.getDuration())
        .withRef(this).withZone(flashCommand.getZones()));
        this.iterations = iterations;
  }

  @Override
  public void initialize() {
    loopIdx = iterations;
  }

  @Override
  public void execute(){
    if(!flashSequence.isScheduled() && loopIdx > 0){
      flashSequence.schedule();
      loopIdx--;
    }
  }

  @Override
  public boolean isFinished(){
    return loopIdx <= 0;
  }

  @Override
  public void end(boolean interrupted) {
    interrupted = interrupted;
  }
  
  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
