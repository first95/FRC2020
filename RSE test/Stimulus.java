import java.util.Random;

public class Stimulus {
    public static final double STIM_DURATION_S = 10;
    private Random rng = new Random();
    private double processStdDev;
    private double measurementStdDev;
    
    /**
     * 
     * @param processVariance squared stddev of process noise (noise in actual process)
     * @param measurementVariance squared stddev of measurement noise (noise in measurement)
     */
    public Stimulus(double processVariance, double measurementVariance) {
        processStdDev = Math.sqrt(processVariance);
        measurementStdDev = Math.sqrt(measurementVariance);
    }

    /**
     * Get the actual value of the physical quantity being estimated
     * @param t time at which to measure
     * @return
     */
    public double GetProcessActual(double t) {
        double noiselessValue = 0;

        // Right now this is just a hard-coded progression
        if( t < 2) {
            noiselessValue = 10 * t;
        } else if (t < 4) {
            noiselessValue = 20;
        } else if (t < 6) {
            noiselessValue = 20-10*(t-4);
        } else if(t < 8) {
            noiselessValue = 20 * (t-6);
        } else {
            noiselessValue = 40;
        }

        double processNoise = processStdDev * rng.nextGaussian();
        return noiselessValue + processNoise;
    }

    /**
     * Get measured version of an actual value
     * @param value actual value
     * @return measured version
     */
    public double Measure(double actual) {
        double measurementNoise = measurementStdDev * rng.nextGaussian();
        return actual + measurementNoise;
    }
}