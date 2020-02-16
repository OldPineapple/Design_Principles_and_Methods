package ca.mcgill.ecse211.lab1;
/**
 * This method states a method and a parameter
 * @author Frank P. Ferrie
 *
 */
public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
