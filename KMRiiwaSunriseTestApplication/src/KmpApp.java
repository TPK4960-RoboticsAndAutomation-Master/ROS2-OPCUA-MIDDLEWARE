
// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


package kmr.test2;

// Configuration
import javax.inject.Inject;
import javax.inject.Named;
import org.apache.log4j.BasicConfigurator;

// Implementated classes
import kmr.test2.KmpCommander;

//RoboticsAPI
import com.kuka.roboticsAPI.annotations.*;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.LBR;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


import com.kuka.generated.ioAccess.ControlPanelIOGroup;

// AUT MODE: 3s, T1/T2/CRR: 2s
@ResumeAfterPauseEvent(delay = 0 ,  afterRepositioning = true)
public class KmpApp extends RoboticsAPIApplication{
	
	// Runtime Variables
	private volatile boolean AppRunning;
	private IAutomaticResumeFunction resumeFunction;
	
	// Declare KMP
	@Inject
	@Named("KMR_omniMove_200_1")
	public KmpOmniMove kmp;
	public Controller controller;

	// Implemented node classes
	KmpCommander kmp_commander;
	
	// Check if application is paused:
	@Inject
	ControlPanelIOGroup ControlPanelIO;

	// Connection types
	String connection = "TCP";

	// Ports
	int kmp_commander_port = 30002;


	public void initialize() {
		System.out.println("Initializing Robotics API Application");

		// Configure application
		BasicConfigurator.configure();
		resumeFunction = getTaskFunction(IAutomaticResumeFunction.class);

		// Configure robot;
		//controller = getController("KUKA_Sunrise_Cabinet_1");
		kmp = getContext().getDeviceFromType(KmpOmniMove.class);		
		
		// Create nodes for communication
		// kmp_commander = new KMP_commander(kmp);
		kmp_commander = new KmpCommander(kmp_commander_port, kmp, connection);

		// Check if a commander node is active
		long startTime = System.currentTimeMillis();
		int shutDownAfterMs = 10000; 
		while(!AppRunning) {
			if(kmp_commander.isSocketConnected()) {
				AppRunning = true;
				System.out.println("Application ready to run!");	
				break;
			} else if((System.currentTimeMillis() - startTime) > shutDownAfterMs) {
				System.out.println("Could not connect to a command node after " + shutDownAfterMs/1000 + "s. Shutting down.");	
				shutdown_application();
				break;
			}				
		}
	}
	
	public void run() {
		setAutomaticallyResumable(true);

		System.out.println("Running app!");
		
		// Start all connected nodes
		kmp_commander.run();
	}

	public void shutdown_application(){
		System.out.println("----- Shutting down Application -----");

		kmp_commander.close();
	
    	System.out.println("Application terminated");
    	    	
    	try {
    		dispose();
    	} catch(Exception e) {
    		System.out.println("Application could not be terminated cleanly: " + e);
    	}
	}
	
	private void setAutomaticallyResumable(boolean enable)
	{
		if(enable)
		{
			resumeFunction.enableApplicationResuming(getClass().getCanonicalName());
			return;
		}
		resumeFunction.disableApplicationResuming(getClass().getCanonicalName());		
	}

	
	public static void main(String[] args){
		KMP_app app = new KMP_app();
		app.runApplication();
	}
	
}