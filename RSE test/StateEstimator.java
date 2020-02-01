public class StateEstimator {
    public StateEstimator() {
        
    }

    /**
     * Get the best estimate of the variable being tracked
     * @param t absolute time (eg, relative to epoch) for which to compute the estimate
     * @return
     */
    public double GetEstimate(double t) {
        return 0; // TODO
    }

    /**
     * Supply a measurement to the estimator
     * @param value output of the sensor
     * @param t absolute time (eg, relative to epoch) at which the measurement was taken, 
     */
    public void AddMeasurement(double value, double t) {
        // TODO
    }
}