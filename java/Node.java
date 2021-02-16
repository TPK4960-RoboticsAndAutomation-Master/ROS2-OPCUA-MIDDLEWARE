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
package kmr.test;

import kmr.test.ISocket;
import kmr.test.TcpSocket;

public abstract class Node extends Thread{
	
	// Runtime Variables
	private volatile static boolean shutdown;
	public volatile boolean closed = false;
	private static volatile boolean EmergencyStop;
	private volatile static boolean PathFinished;
	protected volatile static boolean isKMPmoving;
	private volatile static boolean isKMPconnected;
	private volatile static boolean isLBRconnected;
	private volatile static boolean isLBRmoving;

	// Socket
	protected ISocket socket;
	private String ConnectionType;
	private int port;
	private String remote_ip;
	public static int connection_timeout = 5000;
	
	protected String node_name;
	
	public Node(int port1, String Conn1, int port2, String Conn2, String nodeName){
		this.node_name = nodeName;
		setShutdown(false);
		setEmergencyStop(false);
	}
	
	public Node(String remote_ip, int port, String Conn, String nodeName) {
		this.ConnectionType=Conn;
		this.port = port;
		this.node_name = nodeName;
		this.remote_ip = remote_ip;
		setShutdown(false);
		setEmergencyStop(false);
		
		createSocket();
	}
	
	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			this.socket = new TcpSocket(this.remote_ip, this.port, this.node_name);
		}
	}
	
	public boolean isSocketConnected() {
		return this.socket.isConnected();
	}
	
	public boolean isNodeRunning() {
		return this.isSocketConnected() && (!(this.closed) && !this.getShutdown());
	}
	
	public static void setEmergencyStop(boolean es){
		EmergencyStop = es;
	}
	
	public boolean getEmergencyStop(){
		return EmergencyStop;
	}
	
	public void runmainthread(){
		this.run();
	}
	
	public boolean getShutdown() {
		return shutdown;
	}
	
	public void setShutdown(boolean in) {
		System.out.println("shutdown set by " + this.node_name + " to " + in);
		shutdown=in;
	}
	
	public boolean getisPathFinished() {
        return PathFinished;
	}
	
	public void setisPathFinished(boolean in) {
        PathFinished = in;
	}
	
	public boolean getisKMPMoving() {
        return isKMPmoving;
	}
	
	public void setisKMPMoving(boolean in) {
        isKMPmoving = in;
	}
	
	public boolean getisKMPConnected() {
        return isKMPconnected;
	}
	
	public void setisKMPConnected(boolean in) {
        isKMPconnected = in;
	}

	public boolean getisLBRMoving() {
		return isLBRmoving;
	}
	
	public void setisLBRMoving(boolean in) {
		isLBRmoving = in;
	}
	
	
	public boolean getisLBRConnected() {
		return isLBRconnected;
	}
	
	public void setisLBRConnected(boolean in) {
		isLBRconnected = in;
	}
    
    public abstract void run();
    
    public abstract void close();
}