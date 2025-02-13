package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderSupplier implements Supplier<Encoder> {
  private final int[] port;
  private final double steps;

  private boolean invert = false;

  /** <pre>
   * 
   * @param port encoder ports, this will likely be 2 DIO ports on RIO
   *      write it as 'new int[] {1,2}' for ports 1 and 2
   * @param motionPerCycle the change in location or angle of a mechanism that will equate to one full cycle of the encoder
   *      angle = cycles / gearRatio
   *      location = Final wheel circumference / angle {until the final wheel}
   * 
   *   cycles will be listed on the product page as something like 'Cycles per Revolution'
   *      REV through bore = 2048
   *      AMT103 = 2048
   * 
   *   Final wheel circumference of a gear,sprocket,pully should be based on its pitch diameter
   *      #25 chain = (Teeth * 2.0193  {PD in mm}) * PI
   * </pre>
   */
  public EncoderSupplier(int[] port, double motionPerCycle) {
    this.port = port;
    this.steps = motionPerCycle;
  }

  public EncoderSupplier withInvert() {
    this.invert = true;
    return this;
  }

  public Encoder get() {
    if (port[0] < 0) {
      throw new IllegalStateException("MotorInfo : encoder port 1 < 0, check port is setup");
    }
    if (port[1] < 0) {
      throw new IllegalStateException("MotorInfo : encoder port 2 < 0, check port is setup");
    }
    if (steps < 0) {
      throw new IllegalArgumentException("MotorInfo : EncoderSteps < 0, check EncoderSteps is setup");
    }

    Encoder encoder = new Encoder(port[0], port[1], invert, Encoder.EncodingType.k2X);
    encoder.setDistancePerPulse(steps);
    encoder.setMinRate(0.1 * steps);
    encoder.setMinRate(10);
    encoder.setSamplesToAverage(5);
    return encoder;
  }
}
