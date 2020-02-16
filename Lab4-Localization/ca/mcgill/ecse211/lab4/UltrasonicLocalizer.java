package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Lab4;
import lejos.hardware.Sound;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import ca.mcgill.ecse211.lab4.Odometer;

public class UltrasonicLocalizer {
	private static int d = 31;
	private static int k = 2;

	/**
	 * This is the method for rising edge Detect the first point and do nothing,
	 * then detect the second point and record its angle. Then go back and detect
	 * the third point and do nothing, then the fourth point and record.
	 * 
	 * @throws OdometerExceptions
	 */
	public static void RisingEdge() throws OdometerExceptions {
		//Odometer odometer = Odometer.getOdometer(Lab4.leftMotor, Lab4.rightMotor, Lab4.TRACK, Lab4.WHEEL_RAD);
		Odometer odometer = Odometer.getOdometer();
		Lab4.leftMotor.setSpeed(80);
		Lab4.rightMotor.setSpeed(80);

		// Detect first point
		while (true) {
			// turn clockwise
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
			if (UltrasonicPoller.getDist() < (d - k)) {
				Sound.beep();
				break;
			}
		}

		// Detect second point
		while (true) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
			if (UltrasonicPoller.getDist() > (d + k)) {
				Sound.beep();
				break;
			}
		}
		double angleB = odometer.getXYT()[2];

		// detect third point
		while (true) {
			// turn anticlockwise
			Lab4.leftMotor.backward();
			Lab4.rightMotor.forward();
			if (UltrasonicPoller.getDist() < (d - k)) {
				Sound.beep();
				break;
			}
		}

		// Detect fourth point
		while (true) {
			Lab4.leftMotor.backward();
			Lab4.rightMotor.forward();
			if (UltrasonicPoller.getDist() > (d + k)) {
				Sound.beep();
				break;
			}
		}
		Lab4.leftMotor.stop();
		Lab4.rightMotor.stop();
		double angleA = odometer.getXYT()[2];

		// Calculate the angle that needs to turn to the positive y axis and turn
		// Those values are tested for many times and finally choose them
		if (Math.abs(angleA) < Math.abs(angleB) && angleA < 90) {
			Navigation.anticlockwise(Lab4.WHEEL_RAD, Lab4.WHEEL_RAD, Lab4.TRACK, 215 + angleA / 2 - angleB / 2);
		} 
		else if (Math.abs(angleA) < Math.abs(angleB) && angleA > 90) {
			Navigation.anticlockwise(Lab4.WHEEL_RAD, Lab4.WHEEL_RAD, Lab4.TRACK, 213 + angleA / 2 - angleB / 2);
		} 
		else if (Math.abs(angleA) > Math.abs(angleB) && angleA > 270) {
			Navigation.anticlockwise(Lab4.WHEEL_RAD, Lab4.WHEEL_RAD, Lab4.TRACK, 145 + (angleA - 360) / 2 - 1 / 2 * angleB);
		}
		else if (Math.abs(angleA) > Math.abs(angleB) && angleA < 270) {
			Navigation.anticlockwise(Lab4.WHEEL_RAD, Lab4.WHEEL_RAD, Lab4.TRACK, 125 - 1 / 2 * angleB + 1 / 2 * (angleA - 360));
		}
		odometer.setTheta(0);
	}

	/**
	 * This is the method for falling edge Detect the first point and record its
	 * angle. Then go forward and detect the second point and record its angle.
	 * 
	 * @throws OdometerExceptions
	 */
	public static void FallingEdge() throws OdometerExceptions {
		//Odometer odometer = Odometer.getOdometer(Lab4.leftMotor, Lab4.rightMotor, Lab4.TRACK, Lab4.WHEEL_RAD);
		Odometer odometer = Odometer.getOdometer();
		Lab4.leftMotor.setSpeed(80);
		Lab4.rightMotor.setSpeed(80);
		
		// Detect first point
		while (true) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
			if (UltrasonicPoller.getDist() < (d - k)) {
				Sound.beep();
				break;
			}
		}
		double angleA = odometer.getXYT()[2];

		// Detect second point
		while (true) {
			Lab4.leftMotor.forward();
			Lab4.rightMotor.backward();
			if (UltrasonicPoller.getDist() > (d + k)) {
				Sound.beep();
				break;
			}
		}
		Lab4.leftMotor.stop();
		Lab4.rightMotor.stop();
		double angleB = odometer.getXYT()[2];

		// Calculate the angle needs to turn and turn
		Navigation.clockwise(Lab4.WHEEL_RAD, Lab4.WHEEL_RAD, Lab4.TRACK, 123 + angleA / 2 - angleB / 2);
		odometer.setTheta(0);
	}
}
