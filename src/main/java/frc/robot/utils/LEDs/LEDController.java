package frc.robot.utils.LEDs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDController extends SubsystemBase {
  private class LEDZone {
    public int ID;
    public short[] LEDs;

    public LEDZone(int ID, short[] LEDs) {
      this.ID = ID;
      this.LEDs = LEDs;
    }
  }

  public final AddressableLEDBuffer buffer;

  private final AddressableLED led;
  private boolean updateString = true;
  private final AddressableLEDSim ledSim;

  private static Optional<LEDController> inst = Optional.empty();
  private static List<LEDZone> LEDZones;

  private LEDController() {
    LEDZones = new ArrayList<LEDZone>();

    buffer = new AddressableLEDBuffer(LEDConstants.STRING_LENGTH);
    led = new AddressableLED(LEDConstants.STRING_PORT);
    ledSim = new AddressableLEDSim(led);
    ledSim.setInitialized(true);

    led.setLength(buffer.getLength());
    led.setData(new AddressableLEDBuffer(buffer.getLength()));
    led.start();
  }

  public static LEDController getInstance() {
    if (inst.isEmpty()) {
      inst = Optional.of(new LEDController());
    }

    return inst.get();
  }

  /**  */
  public static LEDController registerZone(int firstLed, int lastLed, int zoneID) {
    return registerZone(new short[] { (short) firstLed }, new short[] { (short) lastLed }, zoneID);
  }

  public static LEDController registerZone(short[] firstLed, short[] lastLed, int zoneID) {
    if (firstLed.length != lastLed.length) {
      throw new IllegalArgumentException("LED : length of firstLed must equal lastLed");
    }
    int length = 0;
    for (int i = 0; i < firstLed.length; i++)
      length += (lastLed[i] - firstLed[i] + 1);
    short[] leds = new short[length];

    int ledsIdx = 0;
    for (int i = 0; i < firstLed.length; i++) {
      for (short j = firstLed[i]; j <= lastLed[i]; j++) {
        leds[ledsIdx++] = j;
      }
    }
    return registerZone(leds, zoneID);
  }

  public static LEDController registerZone(short[] leds, int zoneID) {

    LEDController controller = getInstance();
    for (LEDZone zone : LEDZones) {
      if (zone.ID == zoneID)
        throw new IllegalArgumentException("LED : Duplicate Zone ID");
      for (short ledIn : leds) {
        if (ledIn > controller.buffer.getLength())
          throw new IllegalArgumentException("LED : LED ID out of Range");
        for (short led : zone.LEDs) {
          if (led == ledIn) {
            throw new IllegalArgumentException("LED : LED in multiple zones");
          }
        }
      }
    }

    LEDZones.add(controller.new LEDZone(zoneID, leds));

    return controller;
  }

  public int zoneSize(int id) {
    return LEDZones.get(id).LEDs.length;
  }

  public int zoneStart(int id) {
    return LEDZones.get(id).LEDs[0];
  }

  public short[] getZoneLeds(int id) {
    return Arrays.copyOf(LEDZones.get(id).LEDs, LEDZones.get(id).LEDs.length);
  }

  @Override
  public void periodic() {
    if (updateString) {
      led.setData(buffer);
      updateString = false;
    }
  }

  public void apply(List<Color> LEDColors, int zoneID) {
    LEDZone zone = LEDZones.get(zoneID);
    if (LEDColors.size() != zone.LEDs.length) {
      throw new IllegalArgumentException("LED : apply : send data for all leds in zone(" + zoneID + ")");
    }
    int colorIdx = 0;
    for (short led : zone.LEDs) {
      buffer.setLED(led, LEDColors.get(colorIdx++));
    }

    updateString = true;
  }
}
