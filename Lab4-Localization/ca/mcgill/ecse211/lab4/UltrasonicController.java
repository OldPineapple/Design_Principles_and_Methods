package ca.mcgill.ecse211.lab4;

/**
 * This class states a method and a parameter
 * 
 * @author Group 65
 *
 */
public interface UltrasonicController {

	public void processUSData(int distance);

	public int readUSDistance();
}
