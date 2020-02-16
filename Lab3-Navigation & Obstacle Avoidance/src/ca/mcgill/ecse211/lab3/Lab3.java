package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.Navigation.Odometer;
import ca.mcgill.ecse211.Navigation.Navigation;
import ca.mcgill.ecse211.Navigation.OdometerExceptions;
import ca.mcgill.ecse211.lab3.Display;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * This class operates all codes from all former classes and realize them on the EV3 brick.
 * 
 * @author Group 65
 *
 */
public class Lab3 {
	
	  // Motor Objects, and Robot related parameters.
	  public static final EV3LargeRegulatedMotor leftMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	  public static final EV3LargeRegulatedMotor rightMotor =
	      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
 	  private static final Port usPort = LocalEV3.get().getPort("S1");
	  public static final double WHEEL_RAD = 2.2; // This is the measured distance of wheel radius
	  public static final double TRACK = 17.1; // This is the measured distance from the center of one wheel to that of the other
	  
	  public static void main(String[] args) throws OdometerExceptions {

	    int buttonChoice;
	    
	    // Setup ultrasonic sensor
	    // There are 4 steps involved:
	    // 1. Create a port object attached to a physical port (done already above)
	    // 2. Create a sensor instance and attach to port
	    // 3. Create a sample provider instance for the above and initialize operating mode
	    // 4. Create a buffer for the sensor data
	    @SuppressWarnings("resource") // Because we don't bother to close this resource
	    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	                                                              // this instance
	    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
	                                                         // returned
	    UltrasonicPoller usPoller = null; // the selected controller on each cycle
	    
		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		Display odometryDisplay = new Display(lcd);

		do {
			
		// clear the display
        lcd.clear();

        // ask the user whether only navigate or avoid obstacle
        lcd.drawString("< Left | Right >", 0, 0);
        lcd.drawString("       | Obsta- ", 0, 1);
        lcd.drawString("Navig- | cle    ", 0, 2);
        lcd.drawString(" ation | Avio-  ", 0, 3);
        lcd.drawString("       | dance  ", 0, 4);

        buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
	  } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

      if (buttonChoice == Button.ID_LEFT) {
    	  
        // Display changes in position as wheels are (manually) moved
    	  	Thread odoDisplayThread = new Thread(odometryDisplay);
        odoDisplayThread.start();
        Thread odoThread = new Thread(odometer);
        odoThread.start();
        
        //Run thread of navigation.
   	  	(new Thread() {
	  		public void run() {
	  			Navigation.travelTo();
	  		}
	  	}).start();

      }
      else if(buttonChoice == Button.ID_RIGHT) {
    	  
        // Start odometer and ultrasonic threads
    	    usPoller = new UltrasonicPoller(usDistance, usData);
    	    Thread Ultrasonic = new Thread(usPoller);
    	    Ultrasonic.start();
        Thread odoThread = new Thread(odometer);
        odoThread.start();
        lcd.drawString("US Distance: " + UltrasonicPoller.getDist(), 0, 2);
        
        //Run thread of Obstacle Avoidance.
        (new Thread() {
          public void run() {
        		Avoidance.ProcessUSData();
        	  }
        }).start();
      }
        
      while (Button.waitForAnyPress() != Button.ID_ESCAPE);
      System.exit(0);
    }
}