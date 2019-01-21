// Author: Karl Mason

// This code combines the differential evolution algorithm with novelty search to train a neural network
// controlled robot to navigate through a maze. Two mazes are implemented here.

// Please use the following bib file to cite the paper describing this algorithm, presented at the ALA
// workshop at the 2018 Federated AI Meeting (ICML, AAMAS and IJCAI) in Stockholm, Sweden 

/* 
 
@inproceedings{mason2018maze,
  title={Maze navigation using neural networks evolved with novelty search and differential evolution},
  author={Mason, Karl and Duggan, Jim and Howley, Enda},
  booktitle={Adaptive and Learning Agents Workshop (at ICML-AAMAS 2018)},
  year={2018}
}

 */

package NoveltySearchNeuralNetworkMazeGitHub;

import java.util.ArrayList;
import java.util.Scanner;

public class maze {

	
	int inputSize;
	int outputSize;
	
	ArrayList<Double> solution = new ArrayList<Double>(); // used to record network outputs
	double robotHeading; // degrees anti clockwise from due east, e.g. 90 = north.
	double robotXpos;
	double robotYpos;
	double goalXpos;
	double goalYpos;
	double distance;
	double fitness;
	
	ArrayList<Double> pathX = new ArrayList<Double>();
	ArrayList<Double> pathY = new ArrayList<Double>();
	ArrayList<Double> pathDist = new ArrayList<Double>();
	
	ArrayList<Double> state = new ArrayList<Double>();
	int maxTimeSteps = 200;
	double targetDistance = 0.05;
	double scale = 1; // scale of the map, 1 = 1x1 map size
	double longestDiagonal = Math.sqrt((scale*scale )*2);
	
	ArrayList<ArrayList<Double>> barriers = new ArrayList<ArrayList<Double>>();
	int mazeNum =3; // 3 = normal maze, 2 = hard maze
	
	double maxRotateAngle = 90; // how much the robot can rotate in either direction
	double maxMovementLength = 0.1; // how far the robot can move at a time step
	double barrierThick = 0.00;
	
	double botRad=0.007;
	
	int timeSteps=0;
	
	public maze( ) {
		
		inputSize=10;
		outputSize = 2;
		
		
		distance = Double.POSITIVE_INFINITY;
		
	}


	public void init()
	{
		robotHeading = 90.0;
		if(mazeNum==2){
			robotXpos = 0.1*scale;
			robotYpos = 0.1*scale;
			goalXpos = 0.1*scale;
			goalYpos = 0.9*scale;
			maxTimeSteps = 200;
			targetDistance = 0.05;
			longestDiagonal = Math.sqrt((scale*scale )*2);
		}
		else if(mazeNum==3){
			robotXpos = 0.1*scale;
			robotYpos = 0.7*scale;
			goalXpos = 1.1*scale;
			goalYpos = 0.2*scale;
			maxTimeSteps = 100;
			targetDistance = 0.05;
			longestDiagonal = Math.sqrt((1.2*1.2 )+ (0.8*0.8));
		}
		calcDistance();
		setBarriers();
		pathX = new ArrayList<Double>();
		pathY = new ArrayList<Double>();
		pathDist = new ArrayList<Double>();
		solution = new ArrayList<Double>();
		
	}

	
	public void calcDistance(){
		double xDistSq = Math.pow((robotXpos - goalXpos), 2);
		double yDistSq = Math.pow((robotYpos - goalYpos), 2);
//		System.out.println("Robot x "+robotXpos+", y "+robotYpos);
//		System.out.println("Goal  x "+goalXpos+", y "+goalYpos);
//		System.out.println("xDistSq "+xDistSq+", yDistSq "+yDistSq);
		distance = Math.sqrt(xDistSq + yDistSq);
//		System.out.println("distance "+distance);
	}

	

	
//	void performAction( double[] output ) { // output
	void performAction( ArrayList<Double> output ) { 
		double turnSignal = (-1*(0.5-output.get(0)))/0.5; // -1 = hard left, 1 means hard right
		double turnAngle = turnSignal*maxRotateAngle;
//		System.out.println("output[0] "+output.get(0));
//		System.out.println("turnSignal "+turnSignal);
//		System.out.println("turnAngle "+turnAngle);
//		System.out.println("robotHeading was "+robotHeading);
		double prevHeading = robotHeading;
		
		robotHeading+=turnAngle;
		
	//	System.out.println("robotHeading  is "+robotHeading);
		
		double newHeading = checkHeading(robotHeading);
		robotHeading = newHeading;
		
	//	System.out.println("robotHeading adjusted "+robotHeading);
		
		// use equation of line and equation of a circle to calculate next point for robot
		// centre of circle = h,k.  Radius of circle = r.
		
		double h = robotXpos; 
		double k = robotYpos;
		double r = ((-1*(0.5-output.get(1)))/0.5)*maxMovementLength;
		double M = Math.tan(Math.toRadians(robotHeading)); // slope m = tan (angle)
		
		
		double nextX1 = h+Math.sqrt((r*r)/(1+(M*M))); // using line equation and circle equation
		double nextX2 = h-Math.sqrt((r*r)/(1+(M*M)));
		double nextX;
		double nextY;
		
//		System.out.println("robotHeading "+robotHeading);
//		System.out.println("robotHeading rad "+Math.toRadians(robotHeading));
//		System.out.println("tan robotHeading "+Math.tan(Math.toRadians(robotHeading)));
//		System.out.println("M "+M);
//		System.out.println("cur Robot x "+robotXpos);
//		System.out.println("nextX1 "+nextX1+", nextX2 "+nextX2);
		
		
		if(Double.isInfinite(M)){ // check for errors
			System.out.println("******************************** Infinity **************************");
			
			nextX = robotXpos;
			
			if(Math.abs(90-robotHeading)<Math.abs(270 - robotHeading)){
				nextY = robotYpos + Math.abs(r);
			}
			else{
				nextY = robotYpos - Math.abs(r);
			}
			
		}
		
		
		else{
			if(robotHeading<90 || robotHeading>270){
				nextX=nextX1;
			}
			else{
				nextX=nextX2;
			}
			nextY = ((nextX - h)*M) + k; // using line equation
		}
		
		double x1,y1,x2,y2,x3,y3,x4,y4;
		double M90 = Math.tan(Math.toRadians(robotHeading+90)); // slope m = tan (angle)
		
		if(Double.isInfinite(M)){
			x1= robotXpos - botRad;
			x2= robotXpos + botRad;
			y1= robotYpos;
			y2= robotYpos;
			x3= nextX - botRad;
			x4= nextX + botRad;
			y3= nextY;
			y4= nextY;
		}

		else if(Double.isInfinite(M90)){
			x1= robotXpos;
			x2= robotXpos;
			y1= robotYpos - botRad;
			y2= robotYpos + botRad;
			x3= nextX;
			x4= nextX;
			y3= nextY - botRad;
			y4= nextY + botRad;
		}
		
		else{
			
			x1 = robotXpos+Math.sqrt((botRad*botRad)/(1+(M90*M90))); // using line equation and circle equation
			x2 = robotXpos-Math.sqrt((botRad*botRad)/(1+(M90*M90)));
			x3 = nextX+Math.sqrt((botRad*botRad)/(1+(M90*M90))); // using line equation and circle equation
			x4 = nextX-Math.sqrt((botRad*botRad)/(1+(M90*M90)));
			double C1_2 = robotYpos - (M90*robotXpos);
			double C3_4 = nextY - (M90*nextX);
			
			y1 = M90*x1 + C1_2;
			y2 = M90*x2 + C1_2;
			y3 = M90*x3 + C3_4;
			y4 = M90*x4 + C3_4;
			
		}
		
		// once we have next point, check for barrier collisions
		if(!barrierCollision(h,k,nextX,nextY,false) 
				&& !barrierCollision(x1,y1,x3,y3,false) 
				&& !barrierCollision(x2,y2,x4,y4,false)){
			robotXpos = nextX;// if no collisions, move to the next point
			robotYpos = nextY;
		}
		if(mazeNum==1 ||mazeNum==2){
			if(robotXpos<0 || robotYpos<0 || robotXpos>1 || robotYpos>1){
				System.out.println("****************************** out of bounds *************************");
				System.out.println(" cur (x,y) = ("+robotXpos+", "+robotYpos+")");
				System.out.println(" prev (x,y) = ("+h+", "+k+")");
				System.out.println("output[0] "+output.get(0));
				System.out.println("turnSignal "+turnSignal); 
				System.out.println("turnAngle "+turnAngle);
				System.out.println(" heading = "+robotHeading);
				System.out.println(" prevheading = "+prevHeading);
				System.out.println(" collision = "+barrierCollision(h,k,nextX,nextY,true));
				
				try {
					Thread.sleep(10000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
		
	}
	

	
	boolean barrierCollision(double curX, double curY, double nextX, double nextY, boolean print) {
		boolean collision = false;
		barrierThick=0;
		
		double M1 = (nextY-curY)/(nextX-curX); // y = Mx + C
		double C1 = curY - (M1*curX);
		
		double maxXr;
		double minXr;
		double maxYr;
		double minYr;
		
		if(curX>nextX){
			maxXr = curX;
			minXr = nextX;
		}
		else{
			maxXr = nextX;
			minXr = curX;
		}
		if(curY>nextY){
			maxYr = curY;
			minYr = nextY;
		}
		else{
			maxYr = nextY;
			minYr = curY;
		}
		
		if(Math.abs(M1)>100000){
			barrierThick = 0.00001;
		}
		
		for(int i=0;i<barriers.size();i++){
			boolean tempCol = false;
			
			double x1=barriers.get(i).get(0);
			double y1=barriers.get(i).get(1);
			double x2=barriers.get(i).get(2);
			double y2=barriers.get(i).get(3);
			
			double maxXb;
			double minXb;
			double maxYb;
			double minYb;
			
			if(x1>x2){
				maxXb = x1;
				minXb = x2;
			}
			else{
				maxXb = x2;
				minXb = x1;
			}
			if(y1>y2){
				maxYb = y1;
				minYb = y2;
			}
			else{
				maxYb = y2;
				minYb = y1;
			}
			
			double M2 = (y2-y1)/(x2-x1);
			double C2 = y1 - (M2*x1);
			
			double xIntersect;
			double yIntersect;
			
			

			if(Double.isFinite(M2) && Double.isFinite(M1)){
				xIntersect = (C2-C1)/(M1-M2); 
				yIntersect = (M1*xIntersect) + C1; 
			}
			else if(!Double.isFinite(M2) && Double.isFinite(M1)){
				yIntersect = (M1*x1) + C1;
				xIntersect = (yIntersect-C1)/M1;
			}
			else if(Double.isFinite(M2) && !Double.isFinite(M1)){
				yIntersect = (M2*curX) + C2;
				xIntersect = (yIntersect-C2)/M2;
				
			}
			else{
				xIntersect = (C2-C1)/(M1-M2); 
				yIntersect = (M1*xIntersect) + C1; 
			}

			if(Double.isInfinite(M2)&&!Double.isInfinite(M1)){
					
				if(((xIntersect+barrierThick>=minXr && xIntersect-barrierThick<=maxXr)||(yIntersect+barrierThick>=minYr && yIntersect-barrierThick<=maxYr)) && 
						((yIntersect+barrierThick>=minYb && yIntersect-barrierThick<=maxYb))){
					collision =true;
					tempCol=true;
				}
			}
			
			else if(Double.isInfinite(M1)&&!Double.isInfinite(M2)){
				if((((yIntersect+barrierThick>=minYr && yIntersect-barrierThick<=maxYr)) && 
						((xIntersect+barrierThick>=minXb && xIntersect-barrierThick<=maxXb)||(yIntersect+barrierThick>=minYb && yIntersect-barrierThick<=maxYb))))
				{
					collision =true;
					tempCol=true;
				}
			}
			
			else if(!Double.isInfinite(M1)&&!Double.isInfinite(M2)){
				if(((xIntersect+barrierThick>=minXr && xIntersect-barrierThick<=maxXr)||(yIntersect+barrierThick>=minYr && yIntersect-barrierThick<=maxYr)) && 
						((xIntersect+barrierThick>=minXb && xIntersect-barrierThick<=maxXb)||(yIntersect+barrierThick>=minYb && yIntersect-barrierThick<=maxYb))){
					collision =true;
					tempCol=true;
				}
			}
			else{

			}
			
			
			



			
		}
		if(mazeNum==1 ||mazeNum==2){
			if(nextX<0 || nextY<0 || nextX>1 || nextY>1){
				if(!collision){
					// error
					System.out.println(" cur (x,y) = ("+curX+", "+curY+")");
					System.out.println(" nxt (x,y) = ("+nextX+", "+nextY+")");
					
					try {
						Thread.sleep(10000);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					
				}
			}
		}
		
		if(checkBotRadBarrierCollision(nextX,nextY)){
			return true;
		}
		else{
			return collision;
		}
	}
	
	public boolean checkBotRadBarrierCollision(double nextX, double nextY){
		boolean collision = false;
		double h = nextX; 
		double k = nextY;
		double r = botRad;
		
		barrierThick=0;
		
		for(int i=0;i<barriers.size();i++){
			boolean tempCol = false;
			
			double x1=barriers.get(i).get(0);
			double y1=barriers.get(i).get(1);
			double x2=barriers.get(i).get(2);
			double y2=barriers.get(i).get(3);
			
			double maxXb;
			double minXb;
			double maxYb;
			double minYb;
			
			if(x1>x2){
				maxXb = x1;
				minXb = x2;
			}
			else{
				maxXb = x2;
				minXb = x1;
			}
			if(y1>y2){
				maxYb = y1;
				minYb = y2;
			}
			else{
				maxYb = y2;
				minYb = y1;
			}
			
			double M2 = (y2-y1)/(x2-x1);
			double C2 = y1 - (M2*x1);
			
			
			double xIntersect1;
			double yIntersect1;
			double xIntersect2;
			double yIntersect2;
			
			if(!Double.isFinite(M2)){				// for vertical boundary
				
				if(Math.abs(x1-h)<=r){
					double kpr = k+r;
					double kmr = k-r;
					
					if( (kpr>minYb && kpr<maxYb) || (kmr>minYb && kmr<maxYb) ){
						collision =true;
						tempCol=true;
					}
				}
			}
			else if(Math.abs(M2)<0.0001){		// for hoizontal boundary (0.0001 so not checking = 0 (doubles))
				
				if(Math.abs(y1-k)<=r){
					if(  ((h+r)>minXb && (h+r)<maxXb) || ((h-r)>minXb && (h-r)<maxXb) ){
						collision =true;
						tempCol=true;
					}			
				}
			}
			
			else{
				double dist = Math.abs(C2 + (M2*h) - k)/Math.sqrt(1+(M2*M2));
				if(dist<=r){
					xIntersect1 = h+Math.sqrt((r*r)/(1+(M2*M2))); // using line equation and circle equation
					xIntersect2 = h-Math.sqrt((r*r)/(1+(M2*M2)));
					yIntersect1 = (M2*xIntersect1)+C2;
					yIntersect2 = (M2*xIntersect2)+C2;
					
					if(((xIntersect2+barrierThick>=minXb && xIntersect2-barrierThick<=maxXb)||(yIntersect2+barrierThick>=minYb && yIntersect2-barrierThick<=maxYb)) || 
							((xIntersect1+barrierThick>=minXb && xIntersect1-barrierThick<=maxXb)||(yIntersect1+barrierThick>=minYb && yIntersect1-barrierThick<=maxYb))){
						collision =true;
						tempCol=true;
					}
				}
			}
		
			
		}
		return collision;
	}

	public double evalNet( Network net ) {
		
		double fitness = 0;
		while(solution.size()>0){
			solution.remove(0);
		}
		
		int timeStep=0;
		
		init();		// restart at some point
		net.resetNeuronOutputs();
		boolean stop = false;
		
		while (timeStep < maxTimeSteps && !stop) {
			pathDist.add(distance);
			pathX.add(robotXpos);
			pathY.add(robotYpos);
			ArrayList<Double> netInput = setupInput();
    		ArrayList<Double> netOutput = net.getNetworkOutput(netInput);
    		
    		performAction(netOutput);
    		timeStep++;
    		calcDistance();
    		
    		if(distance<=targetDistance){
    			stop = true;
    			
    			pathDist.add(distance);
    			pathX.add(robotXpos);
    			pathY.add(robotYpos);
   // 			System.out.println("found target ");
//    			System.out.println("Robot x "+robotXpos+", y "+robotYpos);
//    			System.out.println("Goal  x "+goalXpos+", y "+goalYpos);
//    			System.out.println("distance "+distance);
//    			try {
//					Thread.sleep(1000);
//				} catch (InterruptedException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
    		}
    		
		}
		
		timeSteps = timeStep;
		
		solution.add(robotXpos); // solution here is the final position i.e. x, y
		solution.add(robotYpos);
		
		// used in paper
		return 0-distance;
		
//		if(stop){ // if robot reaches target, use fitness function that encourages faster solutions
//			return (0-distance) + (1/timeStep); // may need to use (0.01/timeStep)
//		}
//		else{
//			return 0-distance; // just aim to reach target
//		}
	}
	
	
	
	
	
	public int getInputSize() {
		return inputSize;
	}
	
	public int getOutputSize() {
		return outputSize;
	}
	
	public ArrayList<Double> setupInput(){
		ArrayList<Double> inputs = new ArrayList<Double>();
		
		inputs.add(getNearestBarrierDist(robotXpos,robotYpos,robotHeading)/longestDiagonal); 
		double newHeading = checkHeading(robotHeading-45);
		inputs.add(getNearestBarrierDist(robotXpos,robotYpos,newHeading)/longestDiagonal); 
		newHeading = checkHeading(robotHeading-90);
		inputs.add(getNearestBarrierDist(robotXpos,robotYpos,newHeading)/longestDiagonal); 
		newHeading = checkHeading(robotHeading+45);
		inputs.add(getNearestBarrierDist(robotXpos,robotYpos,newHeading)/longestDiagonal); 
		newHeading = checkHeading(robotHeading+90);
		inputs.add(getNearestBarrierDist(robotXpos,robotYpos,newHeading)/longestDiagonal); 
		newHeading = checkHeading(robotHeading+180);
		inputs.add(getNearestBarrierDist(robotXpos,robotYpos,newHeading)/longestDiagonal); 
		
		// also need to output signal, e.g. 0 1 0 0 for radars to orient robot

		double beta = robotHeading;
		double alpha = Math.abs(Math.atan2(robotYpos - goalYpos,robotXpos - goalXpos)); 
		double ang;
		//maybe check angle alpha...
		
		if(alpha<0){
			System.out.println("ERRROR, alpha < 0 ****************************");
		}
		
		if(alpha>90){
			System.out.println("ERRROR, alpha >90 ----------------------------");
		}
		
		ArrayList<Double> radar = new ArrayList<Double>();
		// 1 in front, 2 left, 3 behind, 4 right
		
		if(goalXpos>=robotXpos && goalYpos<=robotYpos){ // robot is north west of goal
	//		ang = (alpha+beta)%360; 
			ang = checkHeading(alpha+beta);
			if(ang<=45){
				radar = getRadar(1); // in front
			}
			else if(ang<=135){
				radar = getRadar(4); //right
			}
			else if(ang<=225){
				radar = getRadar(3); //behind
			}
			else if(ang<=315){
				radar = getRadar(2); // left
			}
			else{
				radar = getRadar(1); // in front
			}
		}
		else if(goalXpos>=robotXpos && goalYpos>=robotYpos){ // robot is south west of goal
		//	ang = (beta-alpha)%360; 
			ang = checkHeading(beta-alpha);

			if(ang<=45){
				radar = getRadar(1); // in front
			}
			else if(ang<=135){
				radar = getRadar(4); //right
			}
			else if(ang<=225){
				radar = getRadar(3); //behind
			}
			else if(ang<=315){
				radar = getRadar(2); // left
			}
			else{
				radar = getRadar(1); // in front
			}	
		}
		else if(goalXpos>=robotXpos && goalYpos<=robotYpos){ // robot is south east of goal
		//	ang = (alpha+beta)%360; 
			ang = checkHeading(alpha+beta);
			if(ang<=45){
				radar = getRadar(3); // behind
			}
			else if(ang<=135){
				radar = getRadar(2); //left
			}
			else if(ang<=225){
				radar = getRadar(1); //in front
			}
			else if(ang<=315){
				radar = getRadar(4); // right
			}
			else{
				radar = getRadar(3); // behind
			}
		}
		else{ // robot is north east of goal
	//		ang = (beta-alpha)%360; 
			ang = checkHeading(beta-alpha);
			
			if(ang<=45){
				radar = getRadar(3); // behind
			}
			else if(ang<=135){
				radar = getRadar(2); //left
			}
			else if(ang<=225){
				radar = getRadar(1); //in front
			}
			else if(ang<=315){
				radar = getRadar(4); // right
			}
			else{
				radar = getRadar(3); // behind
			}	
		}
			
		for(int i=0;i<radar.size();i++){
			inputs.add(radar.get(i));
		}
		
		return inputs;
	}

	
	
	ArrayList<Double> getRadar(int dir){
		ArrayList<Double> r = new ArrayList<Double>();
		if(dir==1){ // infront
			r.add(1.0);
			r.add(0.0);
			r.add(0.0);
			r.add(0.0);
		}
		else if(dir==2){ // left
			r.add(0.0);
			r.add(1.0);
			r.add(0.0);
			r.add(0.0);
		}
		
		else if(dir==3){ // behind
			r.add(0.0);
			r.add(0.0);
			r.add(1.0);
			r.add(0.0);
		}
		
		else{ // right
			r.add(0.0);
			r.add(0.0);
			r.add(0.0);
			r.add(1.0);
		}
		
		return r;
	}
	
	double checkHeading(double heading){
		double newHeading = heading;
		
		if(newHeading>=360){
			double tempHeading=newHeading%360;
			newHeading = tempHeading;
		}
		if(newHeading<0){
			double tempHeading=newHeading%360;
			newHeading = 360-Math.abs(tempHeading);
		}
		
		return newHeading;
	}
	
	
	public double getNearestBarrierDist(double curX, double curY, double heading) {
		
		double minDist=Double.POSITIVE_INFINITY;
		ArrayList<Double> distances = new ArrayList<Double>();

		double M1 = Math.tan(Math.toRadians(heading)); // slope m = tan (angle)
		double C1 = curY - (M1*curX);
		boolean update = false;
		
		double dist= Double.POSITIVE_INFINITY;
		double xInter = 0;
		double yInter = 0;
		
		for(int i=0;i<barriers.size();i++){
			double x1=barriers.get(i).get(0);
			double y1=barriers.get(i).get(1);
			double x2=barriers.get(i).get(2);
			double y2=barriers.get(i).get(3);
			
			double M2 = (y2-y1)/(x2-x1);
			double C2 = y1 - (M2*x1);
			
			
			if(Double.isFinite(M2) && Double.isFinite(M1)){
				xInter = (C2-C1)/(M1-M2); 
				yInter = (M1*xInter) + C1; 
			}
			else if(!Double.isFinite(M2) && Double.isFinite(M1)){
				yInter = (M1*x1) + C1;
				xInter = (yInter-C1)/M1;
			}
			else if(Double.isFinite(M2) && !Double.isFinite(M1)){
				yInter = (M2*curX) + C2;
				xInter = (yInter-C2)/M2;
			}
			else{
				xInter = (C2-C1)/(M1-M2); 
				yInter = (M1*xInter) + C1; 
			}
		
			dist = getDist(curX,curY,xInter,yInter);
			
			if(heading<90 || heading>270){
				if(xInter>curX){
					if(dist<minDist){
						minDist = dist;
						update = true;
					}
				}	
				
			}
			else if(heading>90 || heading <270){
				if(xInter<curX){
					if(dist<minDist){
						minDist=dist;
						update = true;
					}
				}
			}
			else{
				if(heading>0 && heading<180){
					if(yInter>curY){
						if(dist<minDist){
							minDist=dist;
							update = true;
						}	
					}
				}
				else{
					if(yInter<curY){
						if(dist<minDist){
							minDist=dist;
							update = true;
						}
					}
				}
			}
			
		}
		
		if(!update){
			System.out.println("ERRRRRRRRRROR********************************************************");
			System.out.println("ERRRRRRRRRROR********************************************************");
			System.out.println("ERRRRRRRRRROR********************************************************");
			System.out.println("dist "+dist+", heading "+heading+", minDist "+minDist
					+",\n xinter "+xInter+", yinter "+yInter
					+",\n curX "+curX+", curY "+curY);
			try {
				Thread.sleep(10000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		return minDist;
	}
	
	public double getDist(double x1, double y1,double x2, double y2){
		double t1 = Math.pow((x1-x2), 2);
		double t2 = Math.pow((y1-y2), 2);
		double dist = Math.sqrt(t1+t2);
		return dist;
	}
	
	
	
	public void setBarriers(){
		barriers = new ArrayList<ArrayList<Double>>();
		
		if(mazeNum==2){
			ArrayList<Double> newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(0.0); // left bar
			newBarrier.add(0.0);
			newBarrier.add(1.0);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(1.0);
			newBarrier.add(0.0); // right bar
			newBarrier.add(1.0);
			newBarrier.add(1.0);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(0.0); // bottom bar
			newBarrier.add(1.0);
			newBarrier.add(0.0);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(1.0); // top bar
			newBarrier.add(1.0);
			newBarrier.add(1.0);
			barriers.add(newBarrier);
			
			/// other barriers
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.4);
			newBarrier.add(0.0); //1
			newBarrier.add(0.5);
			newBarrier.add(0.3); // was 0.2
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.3);
			newBarrier.add(0.6); //
			newBarrier.add(0.9); // 
			newBarrier.add(0.2); // 
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.5);
			newBarrier.add(0.6); //
			newBarrier.add(1.0);
			newBarrier.add(0.3);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.3); // 
			newBarrier.add(0.8); // 
			newBarrier.add(0.9); // 
			newBarrier.add(0.9); 
			barriers.add(newBarrier);
			


			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(0.8); //
			newBarrier.add(0.3); //
			newBarrier.add(0.8); // 
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.3);
			newBarrier.add(0.3); // 
			newBarrier.add(0.3); // 
			newBarrier.add(0.8); //
			barriers.add(newBarrier);

			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(0.7); //
			newBarrier.add(0.2);
			newBarrier.add(0.5);
			barriers.add(newBarrier);

			
		}
		
		
		else if(mazeNum==3){
			ArrayList<Double> newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(0.0); // left bar
			newBarrier.add(0.0);
			newBarrier.add(0.8);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(1.2);
			newBarrier.add(0.0); // right bar
			newBarrier.add(1.2);
			newBarrier.add(0.8);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(0.0); // bottom bar
			newBarrier.add(1.2);
			newBarrier.add(0.0);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.0);
			newBarrier.add(0.8); // top bar
			newBarrier.add(1.2);
			newBarrier.add(0.8);
			barriers.add(newBarrier);
			
			/// other barriers
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.3);
			newBarrier.add(0.5); 
			newBarrier.add(0.5);
			newBarrier.add(0.8); 
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.6); // 
			newBarrier.add(0.45); //
			newBarrier.add(0.8); // 
			newBarrier.add(0.8); 
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.9);
			newBarrier.add(0.4); //
			newBarrier.add(1.1);
			newBarrier.add(0.8);
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.2);  
			newBarrier.add(0.4); //
			newBarrier.add(1.0); 
			newBarrier.add(0.0); 
			barriers.add(newBarrier);

			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.6);
			newBarrier.add(0.2); //
			newBarrier.add(0.45); // 
			newBarrier.add(0.55);  //
			barriers.add(newBarrier);
			
			newBarrier = new ArrayList<Double>();
			newBarrier.add(0.9);
			newBarrier.add(0.05); //
			newBarrier.add(0.75); // 
			newBarrier.add(0.4); // 
			barriers.add(newBarrier);

			newBarrier = new ArrayList<Double>();
			newBarrier.add(1.1);
			newBarrier.add(0.0); //
			newBarrier.add(0.95);
			newBarrier.add(0.3);
			barriers.add(newBarrier);

			
		}
		
		
		
		for(int i=0;i<barriers.size();i++){
			for(int j=0;j<barriers.get(i).size();j++){
				double temp = barriers.get(i).get(j);
				barriers.get(i).set(j, (temp*scale));
			}
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}
