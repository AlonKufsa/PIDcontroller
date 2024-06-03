import java.lang.Exception
import kotlin.math.absoluteValue

class PIDController(
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
    private val deltaT: Double = 1.0,
    private val integralMax: Double = 0.0,
) {
	private var integralSum = 0.0
	var tolerance = 0.0
	var error = 0.0
		private set
	private var prevError = 0.0

	/**
	 * Calculates the error as a difference between [setpoint] and [processVariable].
	 *
	 * [setpoint] - The desired value
	 * [processVariable] - the measured value
	 *
	 * If continuous wrapping is enabled then it will calculate the error as the shortest error
	 */
	private fun setError(setpoint: Double, processVariable: Double) {
		error = if (!continuousWrapping)
			setpoint - processVariable
		else {
			if ((setpoint - processVariable).absoluteValue < (((minimumAngle + maximumAngle)/2.0)-minimumAngle)) {
				maximumAngle - (setpoint - processVariable)
			} else setpoint - processVariable
		}
	}

	//**Proportional**
	//Returns a value proportional to the error
	private fun calcProportional(): Double = kP * error


	//**Integral**
	//Adds the current integral to the integral sum (a sum of an approximation of the area under the graph of error over time) and returns a value proportional to the integral sum
	private fun calcIntegral(timeSinceLastMeasurement: Double): Double {
		integralSum += (error * timeSinceLastMeasurement)
		if (integralMax != 0.0 && integralSum >= integralMax) integralSum = 0.0
		return kI * integralSum
	}

	//**Derivative**
	//Calculates the derivative (which in the case of a straight line is just the slope) and returns a value proportional to it
	private fun calcDerivative(timeSinceLastMeasurement: Double): Double {
		val derivative = (error - prevError) / (timeSinceLastMeasurement)
		return kD * derivative
	}

	//A single cycle for the PID
	fun runPIDcycle(setPoint: Double, processVariable: Double, timeSinceLastMeasurement: Double = deltaT): Double {
		setError(setPoint, processVariable)
		val outputPID =
			calcProportional() + calcIntegral(timeSinceLastMeasurement) + calcDerivative(timeSinceLastMeasurement)
		prevError = error
		return outputPID
	}

	fun isWithinTolerance(tol: Double = tolerance) = if (error.absoluteValue < tol) true else false


	//Continuous wrapping
	private var minimumAngle = 0.0
	private var maximumAngle = 0.0
	private var continuousWrapping = false

	fun enableContinuousWrapping(minAngle: Double, maxAngle: Double) {
		if (maxAngle < minAngle) throw Exception("Maximum continuous wrapping angle cannot be smaller than the minimum angle")
		minimumAngle = minAngle
		maximumAngle = maxAngle
		continuousWrapping = true
	}
}