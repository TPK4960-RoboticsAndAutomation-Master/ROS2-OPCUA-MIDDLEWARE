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


import kmr.test2.KMPjogger;

import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;


public class KMP_commander{

	// Robot
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = {0.0,0.0,0.0};
	KMPjogger kmp_jogger;
	long jogging_period  = 20L;

	public KMP_commander(KmpOmniMove robot) {
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp, jogging_period);
	}

	public void run() {
        
        this.kmp_jogger.startJoggingExecution();
        setNewVelocity("lmao 0.05 0 0.1");
		while(true)
		{
			continue;
        }
       
    }
	

	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");

		if(lineSplt.length==4){
				this.velocities[0] =Double.parseDouble(lineSplt[1]); // x
				this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
				this.velocities[2] = Double.parseDouble(lineSplt[3]);  // theta
				
				this.kmp_jogger.updateVelocities(this.velocities);
        }
        
    }
}