/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class LightLocalizer implements Runnable {
	private static int ROTATING_SPEED = 100;
	private double SENSOR_DIST = 13.2;
	double theta[] = new double[4];
	public Odometer odometer;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public LightLocalizer() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}

	Port EV3port = LocalEV3.get().getPort("S4"); // Set the connection port for the sensor
	SensorModes modesensor = new EV3ColorSensor(EV3port); // Get color sensor
	SampleProvider IDSensor = modesensor.getMode("RGB"); // Set sensor mode
	float[] samplecolor = new float[IDSensor.sampleSize()]; // Create data buffer

	/**
	 * Here is where the light sensor and calculating be run.
	 */
	// run method (required for Thread)
	public void run() {
		try {
			Navigation.turnTo(45);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		} // Turn to 45 degrees

		// Let the motor moving forward until it meets a grid line
		Lab4.leftMotor.setSpeed(ROTATING_SPEED);
		Lab4.rightMotor.setSpeed(ROTATING_SPEED);
		Lab4.leftMotor.forward();
		Lab4.rightMotor.forward();

		while (true) {
			IDSensor.fetchSample(samplecolor, 0);
			LCD.drawString("Intensity" + samplecolor[0] * 1000, 0, 4); // This is used to
			// measure the intensity of the black line and background of board
			double samplecolorthou = samplecolor[0] * 1000; // enlarge the value we detected
			
			// The robot beeps and stops when meet the first grid line
			if (samplecolorthou < 120) {
				Sound.beep();
				Lab4.leftMotor.stop();
				Lab4.rightMotor.stop();
				break;
			}
		}

		// Let the robot move backwards a bit in order to make sure the light sensor can
		// detect 4 grid
		// lines in a cycle
		Lab4.leftMotor.setSpeed(ROTATING_SPEED);
		Lab4.rightMotor.setSpeed(ROTATING_SPEED);
		Lab4.leftMotor.rotate(Navigation.convertDistance(Lab4.WHEEL_RAD, -15), true);
		Lab4.rightMotor.rotate(Navigation.convertDistance(Lab4.WHEEL_RAD, -15), false);

		// Detects four lines and record the theta
		int i = 0;
		while (i < 4) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
			IDSensor.fetchSample(samplecolor, 0);
			LCD.drawString("Intensity" + samplecolor[0] * 1000, 0, 4); // This is used to
			// measure the intensity of the black line and background of board
			double samplecolorthou = samplecolor[0] * 1000; // enlarge the value we detected
			if (samplecolorthou < 120) {
				Sound.beep(); // Let the robot beep once we detect the black line
				theta[i] = odometer.getXYT()[2];
				i++;
			}
		}

		Lab4.leftMotor.stop();
		Lab4.rightMotor.stop();

		// Calculating the angle that substends the arc connecting the interestions of
		// the light sensor's
		// path with the axis and calculate the position of
		double thetay = theta[3] - theta[1];
		double thetax = theta[2] - theta[0];
		double dx = -SENSOR_DIST * Math.cos(Math.toRadians(thetay / 2));
		double dy = -SENSOR_DIST * Math.cos(Math.toRadians(thetax / 2));
		odometer.setX(dx);
		odometer.setY(dy);
		try {
			Navigation.travelTo();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
		try {
			Navigation.turnTo(0);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
	}
}
