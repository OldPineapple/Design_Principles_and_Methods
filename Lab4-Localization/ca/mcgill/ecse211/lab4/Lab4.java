package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Odometer;
import ca.mcgill.ecse211.lab4.Navigation;
import ca.mcgill.ecse211.lab4.OdometerExceptions;
import ca.mcgill.ecse211.lab4.Display;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class operates all codes from all former classes and realize them on the
 * EV3 brick.
 * 
 * @author Group 65
 *
 */
public class Lab4 {

	// Motor Objects, and Robot related parameters.
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S1");
	public static final double WHEEL_RAD = 2.2; // This is the measured distance of wheel radius
	public static final double TRACK = 17.1; // This is the measured distance from the center of one wheel to that of
												// the other

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Setup ultrasonic sensor
		// There are 4 steps involved:
		// 1. Create a port object attached to a physical port (done already above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating
		// mode
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
		LightLocalizer LightLocalizer = new LightLocalizer();

		do {

			// clear the display
			lcd.clear();

			// ask the user whether use rising edge or falling edge
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising | Falling", 0, 2);
			lcd.drawString("Edge   | Edge   ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {

			// Start odometer and ultrasonic and display threads
			usPoller = new UltrasonicPoller(usDistance, usData);
			Thread Ultrasonic = new Thread(usPoller);
			Ultrasonic.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			// Run thread of rising edge.
			(new Thread() {
				public void run() {
					try {
						UltrasonicLocalizer.RisingEdge();
					} catch (OdometerExceptions e) {
						e.printStackTrace();
					}
				}
			}).start();
		} else if (buttonChoice == Button.ID_RIGHT) {

			// Start odometer and ultrasonic and display threads
			usPoller = new UltrasonicPoller(usDistance, usData);
			Thread Ultrasonic = new Thread(usPoller);
			Ultrasonic.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			// Run thread of falling edge.
			(new Thread() {
				public void run() {
					try {
						UltrasonicLocalizer.FallingEdge();
					} catch (OdometerExceptions e) {
						e.printStackTrace();
					}
				}
			}).start();
		}

		while (Button.waitForAnyPress() != Button.ID_ENTER)
			;// wait for any button press
		
		// Start light sensor thread

		Thread LightLocalizerThread = new Thread(LightLocalizer);
		LightLocalizerThread.start();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}