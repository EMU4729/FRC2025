package frc.robot.LEDs;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.util.Color;

public class RainbowLEDCommand extends LEDCommand {
  private int firstHue = 0;
  private final int hueStep = 2;
  private int delayIdx = 0;
  private final int delay;

  public RainbowLEDCommand() {
    this.delay = 3;
  }
  public RainbowLEDCommand(int delay){
    this.delay = delay;
  }

  @Override
  public void execute() {
    if(delayIdx++ > delay){delayIdx = 0;}
    else{return;}
    for(LEDZone zone : zones){
      List<Color> colors = new ArrayList<>(zone.zoneSize());
      short[] leds = zone.getLedIds();
      for (short led : leds) {
        colors.add(Color.fromHSV((firstHue+led*hueStep)%180, 255, 128));
      }
      
      // Increase by to make the rainbow "move"
      
      zone.apply(colors);
    }
    firstHue = (firstHue+hueStep)%180;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
