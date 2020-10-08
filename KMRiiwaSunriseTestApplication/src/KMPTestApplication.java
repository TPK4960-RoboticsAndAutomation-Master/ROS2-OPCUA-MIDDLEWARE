package KMRiiwaSunriseTestApplication.src;

//RoboticsAPI
import com.kuka.roboticsAPI.annotations.*;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


import com.kuka.generated.ioAccess.ControlPanelIOGroup;

import API_ROS2_Sunrise.KMPjogger;
import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;

// AUT MODE: 3s, T1/T2/CRR: 2s
@ResumeAfterPauseEvent(delay = 0 ,  afterRepositioning = true)
public class KMPTestApplication extends RoboticsAPIApplication{
    // Runtime Variables
    private volatile boolean AppRunning;
    private IAutomaticResumeFunction resumeFunction;
    SafetyStateListener safetylistener;

    // Declare KMP
    @Inject
    @Named("KMR_omniMove_200_1")
    public KmpOmniMove kmp;
    public Controller controller;

    // Check if application is paused:
    @Inject
    ControlPanelIOGroup ControlPanelIO;

    long jogging_period  = 1L;

    public KMPjogger kmpJogger = new KMPjogger((ICartesianJoggingSupport)kmp, jogging_period);
    double[] velocities = {0.0,0.0,10.0};

    public void initialize() {
        System.out.println("Initializing Robotics API Application");

        // Configure application
        BasicConfigurator.configure();
        resumeFunction = getTaskFunction(IAutomaticResumeFunction.class);

        // Configure robot;
        controller = getController("KUKA_Sunrise_Cabinet_1");
        kmp = getContext().getDeviceFromType(KmpOmniMove.class);		
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
        KMRiiwaSunriseApplication app = new KMRiiwaSunriseApplication();
        app.runApplication();

        while(true){
            app.kmpJogger.updateVelocities(app.velocities);
        }
    }

}