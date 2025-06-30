package frc.robot.utils.LEDs;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem controls the robot's LEDs.
 * 
 * To use, instantiate, and then modify the sub's public `buffer` field (an
 * instance of {@link AddressableLEDBuffer}) in a command, and then call
 * `apply`.
 */
public class LEDZone extends SubsystemBase {
  private int id;

  public LEDZone(int firstLed, int lastLed, int id) {
    LEDController.registerZone(firstLed, lastLed, id);
    this.id = id;
  }
  public LEDZone(short[] firstLeds, short[] lastLeds, int id) {
    LEDController.registerZone(firstLeds, lastLeds, id);
    this.id = id;
  }
  public LEDZone(short[] leds, int id) {
    LEDController.registerZone(leds, id);
    this.id = id;
  }

  /** Applies the data in the buffer to the LEDs */
  public void apply(List<Color> colors) {
    LEDController.getInstance().apply(colors, id);
  }
  public void apply(Color color) {
    LEDController ctrlr = LEDController.getInstance();
    List<Color> colors = new ArrayList<Color>(Collections.nCopies(ctrlr.zoneSize(id), color));
    ctrlr.apply(colors, id);
  }

  public int zoneSize(){
    return LEDController.getInstance().zoneSize(id);
  }
  public int zoneStart(){
    return LEDController.getInstance().zoneStart(id);
  }
  public short[] getLedIds(){
    return LEDController.getInstance().getZoneLeds(id);
  }
  public int getID(){return id;}
}
