/*
 * PowerDistributionPanelWrapper
 * 
 * This class wraps the PowerDistributionPanel class, and offers singleton functionality
 */
package frc.robot.components;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerDistributionPanelWrapper extends PowerDistributionPanel {

	/**
	 * Make the constructor private
	 */
	private PowerDistributionPanelWrapper() {
		super();
	}

	/** 
	 * Enforce singleton behavior
	 */
	private static PowerDistributionPanelWrapper instance;
	public static PowerDistributionPanelWrapper Instance() {
		if(instance == null) {
			instance = new PowerDistributionPanelWrapper();
		}
		return instance;
	}
}
