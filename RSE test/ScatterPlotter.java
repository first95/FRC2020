
import java.awt.Color;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

/**
 * Create this object, give it several series, "populate" it, then show it
 */
public class ScatterPlotter extends JFrame {
    private static final long serialVersionUID = 6294689542092367724L;

    private XYSeriesCollection dataset = new XYSeriesCollection();

    public ScatterPlotter(String windowTitle) {
        super(windowTitle);

    }

    /**
     * Set up chart for showing.
     * Call this after adding all your series but before showing this window.
     * @param plotTitle Mandatory because people who don't label their plots like to murder kittens
     * @param xAxisTitle Mandatory because people who don't label their axes are the worst people ever
     * @param yAxisTitle Mandatory because people who don't label their axes are the kind of people who kick puppies
     */
    public void PopulateChart(String plotTitle, String xAxisTitle, String yAxisTitle) {

        // Create chart
        JFreeChart chart = ChartFactory.createScatterPlot(plotTitle, xAxisTitle, yAxisTitle, dataset);

        // Changes background color
        XYPlot plot = (XYPlot) chart.getPlot();
        plot.setBackgroundPaint(new Color(255, 228, 196));

        // Create Panel
        ChartPanel panel = new ChartPanel(chart);
        setContentPane(panel);
    }

    /**
     * Add a series to this plot. Call before calling PopulateChart
     * @param series
     */
    public void AddSeries(XYSeries series) {
        dataset.addSeries(series);
    }


    public static void main(String[] args) {
        ScatterPlotter sp = new ScatterPlotter("Scatter plotter test");
        
        // Boys (Age,weight) series
        XYSeries series1 = new XYSeries("Boys");
        series1.add(1, 72.9);
        series1.add(2, 81.6);
        series1.add(3, 88.9);
        series1.add(4, 96);
        series1.add(5, 102.1);
        series1.add(6, 108.5);
        series1.add(7, 113.9);
        series1.add(8, 119.3);
        series1.add(9, 123.8);
        series1.add(10, 124.4);

        sp.AddSeries(series1);

        // Girls (Age,weight) series
        XYSeries series2 = new XYSeries("Girls");
        series2.add(1, 72.5);
        series2.add(2, 80.1);
        series2.add(3, 87.2);
        series2.add(4, 94.5);
        series2.add(5, 101.4);
        series2.add(6, 107.4);
        series2.add(7, 112.8);
        series2.add(8, 118.2);
        series2.add(9, 122.9);
        series2.add(10, 123.4);

        sp.AddSeries(series2);

        sp.PopulateChart("test", "X", "weight");

        SwingUtilities.invokeLater(() -> {
            sp.setSize(800, 400);
            sp.setLocationRelativeTo(null);
            sp.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            sp.setVisible(true);
    });
  }
}