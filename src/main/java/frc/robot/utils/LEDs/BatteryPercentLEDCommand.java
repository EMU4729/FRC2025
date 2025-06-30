package frc.robot.utils.LEDs;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;

public class BatteryPercentLEDCommand extends Command {
  static PowerDistribution pdb = new PowerDistribution();
  boolean finished = false; 
  Command curentCommand = null;
  
  private void flash(Color color, int duration, int iterations){
    curentCommand = new RepeatedFlashLEDCommand((FlashSolidLEDCommand)(
            new FlashSolidLEDCommand(color, duration)
            .withZone(new int[]{0})), iterations
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    curentCommand.schedule();
  }
  private void hold(Color color){
    curentCommand = new SolidLEDCommand(color)
        .withZone(new int[]{0})
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    curentCommand.schedule();
  }

  private void checkBattery(){
    if(curentCommand != null){
      curentCommand.cancel();
      curentCommand = null;
    }
    double voltage = pdb.getVoltage();
    if(voltage < 11){
      flash(Color.kRed, 167, 1000000);
    } else if(voltage < 12.25){
      voltage = (voltage-11)/2;
      flash(Color.fromHSV(30, 255, LEDConstants.MAX_SUSTAINED_BRIGHTNESS), (int)(voltage*1833+167), 1000000);

      //flash(Color.kDarkGreen, 1000, 1000000);
    } else if(voltage < 13){
      voltage = (voltage-11.75)/1.25;
      flash(Color.fromHSV(60, 255, LEDConstants.MAX_SUSTAINED_BRIGHTNESS), (int)(voltage*1833+167), 1000000);

      //flash(Color.kDarkGreen, 1000, 1000000);
    } else {
      hold(Color.fromHSV(60, 255, LEDConstants.MAX_SUSTAINED_BRIGHTNESS));
    }
  }

  @Override
  public void initialize() {
    finished = false;
  }

  int executeIdx = 3001;
  @Override
  public void execute() {
    if(executeIdx++ > 3000){      
      checkBattery();
      executeIdx = 0;
    }
    
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  public void finish() {
    finished = true;
    if(curentCommand != null){curentCommand.cancel();}
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}