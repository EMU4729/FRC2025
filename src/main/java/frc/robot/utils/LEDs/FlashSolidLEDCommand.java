package frc.robot.utils.LEDs;

import java.time.Instant;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;

public class FlashSolidLEDCommand extends SolidLEDCommand {
  private Instant endTime;
  private final int msDuration;

  /**
   * 
   * @param color
   * @param msDuration must be >167
   */
  public FlashSolidLEDCommand(Color color, int msDuration){
    super(color);
    this.msDuration = Math.max(msDuration, 1/LEDConstants.MAX_FLASH_RATE); //anti epilepsy
  }
  
  @Override
  public void initialize() {
    super.initialize();
    endTime = Instant.now().plusMillis(msDuration);
  }

  
  @Override
  public boolean isFinished() {
    return Instant.now().isAfter(endTime);
  }

  public int getDuration(){return msDuration;}
}
