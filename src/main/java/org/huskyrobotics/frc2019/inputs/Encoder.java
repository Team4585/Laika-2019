package org.huskyrobotics.frc2019.inputs;

public class Encoder{
    public static enum EncoderMode {
        None, QuadEncoder, CTRE_MagEncoder_Relative, CTRE_MagEncoder_Absolute
    }
  
    public static double rawToRotation(int rawPosition, double unitsPerRotation) {
		double rotations = (rawPosition / unitsPerRotation);
		return rotations;
	}

	/**
	 * Converts raw sensor position to rotations. Units are full rotations of the object.
	 * 
	 * @param rotations
	 * @param unitsPerRotation
	 * @return rawPosition
	 */
	public static double rotationsToRaw(double rotations, double unitsPerRotation) {
		return rotations * unitsPerRotation;
	}

	/**
	 * Converts rotations to physical distance. Units are the same as the radius.
	 * @param rotations
	 * @param effectiveDiam
	 * @return distance
	 */
	public static double rotationsToDistance(double rotations, double effectiveDiam) {
		double circumference = Math.PI * effectiveDiam;
		double distance = circumference * rotations;
		return distance;
	}

	/**
	 * Converts distance to rotations. Units are the same as the radius.
	 * @param distance
	 * @param effectiveDiam
	 * @return rotations
	 */
	public static double distanceToRotations(double distance, double effectiveDiam) {
		double circumference = Math.PI * effectiveDiam;
		double rotations = distance / circumference;
		return rotations;
	}

	/** Converts a raw position (delta or absolute) to a distance.
	 * @param rawPosition
	 * @param unitsPerRotation
	 * @param effectiveDiam
	 * @return distance
	 * 
	 */
	public static double rawToDistance(double rawPosition, double unitsPerRotation, double effectiveDiam) {
		double rotations = rawPosition / unitsPerRotation;
		double circumference = Math.PI * effectiveDiam;
		double distance = rotations * circumference;
		return distance;
	}

	/** Converts a distance to raw position (delta or absolute)
	 * @param distance in feet
	 * @param effectiveDiam in feet
	 * @param unitsPerRotation in raw encoder units
	 * @return rawPosition in feet
	 */
	public static double distanceToRaw(double distance, double effectiveDiam, double unitsPerRotation) {

		double rawDistance = ((distance) / (Math.PI * effectiveDiam)) * unitsPerRotation;

		return rawDistance;
	}

	/**
	 * Converts raw distance to degrees
	 * @param rawPosition
	 * @param unitsPerRotation
	 * @return degrees
	 */
	public static double rawToDegrees(int rawPosition, double unitsPerRotation) {
		return rawToRotation(rawPosition, unitsPerRotation) * 360;
	}

	/**
	 * Converts degrees to raw distance
	 * @param degrees
	 * @param unitsPerRotation
	 * @return rawPosition
	 */
	public static double degreesToRaw(double degrees, double unitsPerRotation) {
		double rotations = degrees / 360;
		double raw = rotations * unitsPerRotation;
		return raw;
	}
        None, QuadEncoder, CTRE_MagEncoder_Absolute, CTRE_MagEncoder_Relative
    }
}