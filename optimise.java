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

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;


public abstract class optimise {
	
	int trainingStatesNo;
	maze mprob = new maze();
	boolean doNotRun = false;
	double maxI;
	ArrayList<Integer> ViolationCounts = new ArrayList<Integer>();
	ArrayList<Double> Violations = new ArrayList<Double>();
	ArrayList<Double> Fitness = new ArrayList<Double>();

	
	ArrayList<ArrayList<Double>> fsolutions = new ArrayList<ArrayList<Double>> ();
	ArrayList<ArrayList<Double>> bsolutions = new ArrayList<ArrayList<Double>> ();
	Network network;
	public double bestFitness;
	double bestViolationSum;
	int bestViolationCount;
	double novelty;
	int k=15;
	ArrayList<ArrayList<Double>> solnBehave = new ArrayList<ArrayList<Double>>();
	ArrayList<ArrayList<Double>> archiveSolnBehave = new ArrayList<ArrayList<Double>>();
	ArrayList<Double> archiveNoveltyScores = new ArrayList<Double>();
	ArrayList<Double> noveltyScores = new ArrayList<Double>();
	ArrayList<Double> normalNoveltyScores = new ArrayList<Double>();
	int maxNovArchiveSize = k*1;
	
	

	ArrayList<Double> bestMazePathX = new ArrayList<Double>();
	ArrayList<Double> bestMazePathY = new ArrayList<Double>();
	ArrayList<Double> bestMazePathD = new ArrayList<Double>();
	ArrayList<Double> finalRobotXPos = new ArrayList<Double>();
	ArrayList<Double> finalRobotYPos = new ArrayList<Double>();
	double bestMazeDist;
	double bestMazeTime;
	
	
	
	ArrayList <Double> optimumNetworkWeights;
	ArrayList <Double> prevGenWeights;
	
	double currentViolationSum;
	int currentViolationCount;
	double currentCostSum = 0.0;
	double currentEmissionsSum = 0.0;
	
		
	ArrayList<Double> solution = new ArrayList<Double>(); // used to record network outputs
	ArrayList<Double> curGenObjectiveFitnesses = new ArrayList<Double>(); // used to record network outputs
	ArrayList<Double> curGenNormalObjectiveFitnesses = new ArrayList<Double>();
	
	ArrayList<Double> curGenSteps = new ArrayList<Double>();
	ArrayList<Integer> curGenViolationCount = new ArrayList<Integer>();
	ArrayList<Double> curGenViolationSum = new ArrayList<>();
	
	ArrayList<ArrayList<Double>> curGenMazePathX = new ArrayList<ArrayList<Double>>();
	ArrayList<ArrayList<Double>> curGenMazePathY = new ArrayList<ArrayList<Double>>();
	ArrayList<ArrayList<Double>> curGenMazePathD = new ArrayList<ArrayList<Double>>();
	ArrayList<Double> curGenMazeDist = new ArrayList<Double>();
	ArrayList<Double> curGenMazeTime = new ArrayList<Double>();
	
	
	double currentOverallFitness;
	public double totalTestFitness;
	double testViolationSum;
	double testViolationCount;
	public ArrayList<Double> testFitness = new ArrayList<Double>();
	public ArrayList<Integer> networkTopology = new  ArrayList<Integer>();
	public int numConnections;
	public int evals=0;
	
	public int maxEval;
	
	double steps =0;
	double bestSteps = 0;
	boolean mazeProblem = true;
	boolean stop=false;
	boolean improved = false;
	
	int generality=0;
	
	boolean noveltySearch = true;
	double ro = 1.0; // 1 = novelty, 0 = fitness
	double noveltyDecay = 1.0;
	
	public optimise (){
		
	}
	
	public optimise (ArrayList<Integer> networkStructure,ArrayList<Double> prevGenWeights, 
			double par , int evals){
		this.prevGenWeights = prevGenWeights;
		
		this.noveltyDecay=par;
		this.currentOverallFitness=0.0;
		this.currentViolationCount=0;
		this.currentViolationSum=0.0;
		this.mprob = new maze();
		
		if(mazeProblem){
			this.mprob = new maze();
			this.network = new Network(mprob.inputSize,mprob.outputSize,networkStructure); 
		}
		
		this.optimumNetworkWeights = new ArrayList <Double> ();
		this.numConnections = prevGenWeights.size();
		
		if(this.numConnections<=0){
			doNotRun = true;
		}
		
		this.maxI=evals;
		for(int i=0;i<prevGenWeights.size();i++){
			networkTopology.add(1);
		}
	}
	
	
	public abstract void trainNetwork();
	
	
	public ArrayList<Double> positionToWeights(ArrayList<Double> Position){
		ArrayList<Double> Weights = new ArrayList<Double>();
		int indx=0;
		for(int i=0;i< networkTopology.size();i++){
			if(networkTopology.get(i)==0){
				Weights.add(0.0);
			}
			else{
				Weights.add(Position.get(indx));
				indx++;
			}
		}
		
		return Weights;
	}
	
	public ArrayList<Double> weightToPosition(ArrayList<Double> Weight){
		ArrayList<Double> Position = new ArrayList<Double>();
		
		for(int i=0;i< networkTopology.size();i++){
			if(networkTopology.get(i)==1){
				Position.add(Weight.get(i));
			}
		}
		
		return Position;
	}
	
	
	public int getNumberNetworkConnections(ArrayList<Integer> networkTopology){
		int num=0;
		for(int i=0;i< networkTopology.size();i++){
			if(networkTopology.get(i)==1){
				num++;
			}
		}
		return num;
	}
	
	
	public void setNetworkWeights(String path){
		ArrayList<Double> weights = readFile(path);
		network.setWeights(weights);
	}
	
	public ArrayList<Double> readFile(String path){
		ArrayList<Double> contents = new ArrayList<Double>();
		
		try {
			Scanner s = null; 
			s = new Scanner(new BufferedReader(new FileReader(path))); 
			while (s.hasNext()) { 
				contents.add(s.nextDouble()); // next integer is added to randomList
				}
			s.close();
			
		} catch (FileNotFoundException e) {
			
			e.printStackTrace();
		}
		return contents;
		
	}
	
	

	
	public void updateBestFitness(){
		improved=true;
		bsolutions=fsolutions;
		this.bestSteps = steps;
		bestFitness = currentOverallFitness;
		bestViolationCount = currentViolationCount;
		bestViolationSum = currentViolationSum;
		optimumNetworkWeights = network.getWeights();
		
	}
	
	
	public void updateBestFitness(int indx){
		improved=true;
		bsolutions=fsolutions;
		this.bestSteps = curGenSteps.get(indx); 
		bestFitness = curGenObjectiveFitnesses.get(indx); 
		bestViolationCount = curGenViolationCount.get(indx); 
		bestViolationSum = curGenViolationSum.get(indx);
		optimumNetworkWeights = network.getWeights();
		
		if(mazeProblem){
			
			bestMazePathX = new ArrayList<Double>();
			bestMazePathX = curGenMazePathX.get(indx); 
			bestMazePathY = new ArrayList<Double>();
			bestMazePathY = curGenMazePathY.get(indx);
			bestMazePathD = new ArrayList<Double>();
			bestMazePathD = curGenMazePathD.get(indx);
			bestMazeDist = curGenMazeDist.get(indx);
			bestMazeTime = curGenMazeTime.get(indx);
		}
		
	}
	
	
	
	
	public void addBestValues(){
		ViolationCounts.add(bestViolationCount);
		Violations.add(bestViolationSum);
		Fitness.add(bestFitness);
	}
	
	public void evaluateCurNetwork(){
		while(solution.size()>0){
			solution.remove(0);
		}
		
		if(mazeProblem){
			currentOverallFitness = mprob.evalNet(network);
			finalRobotXPos.add(mprob.robotXpos);
			finalRobotYPos.add(mprob.robotYpos);
//			System.out.println("current fit = "+currentOverallFitness);
		}
		
		curGenObjectiveFitnesses.add(currentOverallFitness);
		// add values
		curGenMazePathX.add(mprob.pathX);
		curGenMazePathY.add(mprob.pathY);
		curGenMazePathD.add(mprob.pathDist);
		curGenMazeDist.add(mprob.distance);
		curGenMazeTime.add((double)mprob.timeSteps);
		
		curGenSteps.add(this.steps);
		curGenViolationCount.add(currentViolationCount);
		curGenViolationSum.add(currentViolationSum);


		addBehave();
		
	}
	
	public void deleteBehave(){
		
		// reset values
		
		while(curGenSteps.size()>0){
			curGenSteps.remove(0);
		}
		
		while(curGenViolationCount.size()>0){
			curGenViolationCount.remove(0);
		}
				
		while(curGenViolationSum.size()>0){
			curGenViolationSum.remove(0);
		}
		
		while(curGenMazePathX.size()>0){
			curGenMazePathX.remove(0);
		}
		while(curGenMazePathY.size()>0){
			curGenMazePathY.remove(0);
		}
		while(curGenMazePathD.size()>0){
			curGenMazePathD.remove(0);
		}
		while(curGenMazeDist.size()>0){
			curGenMazeDist.remove(0);
		}
		while(curGenMazeTime.size()>0){
			curGenMazeTime.remove(0);
		}
		
		///////////////////
		
		while(solnBehave.size()>0){
			solnBehave.remove(0);
		}
		while(curGenObjectiveFitnesses.size()>0){
			curGenObjectiveFitnesses.remove(0);
		}
		
	}
	
	public void addBehave(){
		
		if (mazeProblem){
			ArrayList <Double> temp = new ArrayList <Double> ();
			for(int i=0;i<mprob.solution.size();i++){
				temp.add(mprob.solution.get(i)); 
			}
			solnBehave.add(temp);
		}
	}
	
	public void calcNovelty(){

		while(noveltyScores.size()>0){
			noveltyScores.remove(0);
		}
		while(normalNoveltyScores.size()>0){
			normalNoveltyScores.remove(0);
		}
		while(curGenNormalObjectiveFitnesses.size()>0){
			curGenNormalObjectiveFitnesses.remove(0);
		}
		
		double Nmax = Double.NEGATIVE_INFINITY;
		double Nmin = Double.POSITIVE_INFINITY;
		
		
		for(int i=0;i<solnBehave.size();i++){
			double temp=0.0;
	//		System.out.println("Behave = "+solnBehave.get(i));
			
			ArrayList <Double> dist = new ArrayList <Double> ();
			
			dist = sortNearNeighbours(i,solnBehave);
			
			
			
			if(k<=dist.size()){
				for(int j=0;j<k;j++){
					temp=temp+dist.get(j);
				}
			}
			else{
				for(int j=0;j<dist.size();j++){
					temp=temp+dist.get(j);
				}
			}
			if(temp>Nmax){
				Nmax = temp;
			}
			if(temp<Nmin){
				Nmin = temp;
			}
			
			noveltyScores.add(temp);
		}
		
		for(int i=0;i<noveltyScores.size();i++){
			double ntemp = (noveltyScores.get(i)- Nmin)/(Nmax-Nmin);
			normalNoveltyScores.add(ntemp);		
		}
		
		
		
		double Fmax = Double.NEGATIVE_INFINITY;
		double Fmin = Double.POSITIVE_INFINITY;
		
		for(int i=0;i<curGenObjectiveFitnesses.size();i++){
			if(curGenObjectiveFitnesses.get(i)>Fmax){
				Fmax =curGenObjectiveFitnesses.get(i);
			}
			if(curGenObjectiveFitnesses.get(i)<Fmin){
				Fmin =curGenObjectiveFitnesses.get(i);
			}
		}
		for(int i=0;i<curGenObjectiveFitnesses.size();i++){
			double ftemp = (curGenObjectiveFitnesses.get(i)- Fmin)/(Fmax-Fmin);
			curGenNormalObjectiveFitnesses.add(ftemp);
			
		}
		
	}
	
	public ArrayList <Double> sortNearNeighbours(ArrayList <Double> curS, 
			ArrayList <ArrayList <Double>> neighbours){		
		
		ArrayList <Double> dist = new ArrayList <Double> ();
		ArrayList <Double> curBehave = curS;
		
		for(int i=0;i<neighbours.size();i++){
			double d=0.0;
			double d2=0.0;
			
			for(int j=0;j<neighbours.get(i).size();j++){
				d=d+ Math.pow( (curBehave.get(j)- neighbours.get(i).get(j)), 2);
			}
			d2=d*d;
			dist.add(d2);
			
		}
		
		return sortLowToHigh(dist);
	}
	
	public ArrayList <Double> sortNearNeighbours(int indx, ArrayList <ArrayList <Double>> neighbours){		
		ArrayList <Double> dist = new ArrayList <Double> ();
		ArrayList <Double> curBehave = neighbours.get(indx);
		
		for(int i=0;i<neighbours.size();i++){
			double d=0.0;
			double d2=0.0;
			if(i!=indx){
				for(int j=0;j<neighbours.get(i).size();j++){
					d=d+ Math.pow( (curBehave.get(j)- neighbours.get(i).get(j)), 2);
				}
				d2=d*d;
				dist.add(d2);
			}
		}
		
		return sortLowToHigh(dist);
	}
	
	
	public ArrayList <Double> sortLowToHigh(ArrayList <Double> list){ // returns low to high
	//	System.out.println("pre sort "+list);
		ArrayList <Double> sortList = list;
		int minPos = 0;
		double tempMin;
		double tempI;
        for(int i=0; i< sortList.size(); i++){
        	minPos = i;
        	for(int j=i; j< sortList.size(); j++){
        		if(sortList.get(j) < sortList.get(minPos)){
        			minPos = j;
        		}
  
        	}
        	tempMin=sortList.get(minPos);
        	tempI=sortList.get(i);
        	sortList.set(minPos, tempI);
        	sortList.set(i, tempMin);
        	
        }
  //      System.out.println("aft sort "+sortList+"\n\n");
        
        return sortList;
	}
	
	
	public ArrayList <Double> sortHighToLow(ArrayList <Double> list){ // returns high to low
		ArrayList <Double> sortList = list;
		int maxPos = 0;
		double temp;
        for(int i=0; i< sortList.size(); i++){
        	maxPos = i;
        	for(int j=i; j< sortList.size(); j++){
        		if(sortList.get(j) > sortList.get(maxPos)){
        			maxPos = j;
        		}
  
        	}
        	temp=sortList.get(maxPos);
        	sortList.set(maxPos, sortList.get(i));
        	sortList.set(i, temp);
        	
        }
        
        return sortList;
	}
	
	public ArrayList <ArrayList <Double>> sortHighToLow(ArrayList <Double> list, ArrayList <ArrayList <Double>> list2){ // returns high to low
		ArrayList <Double> sortList = list;
		ArrayList <ArrayList <Double>> returnList = new ArrayList <ArrayList <Double>>();
		
		int maxPos = 0;
		double temp;
        for(int i=0; i< sortList.size(); i++){
        	maxPos = i;
        	for(int j=i; j< sortList.size(); j++){
        		if(sortList.get(j) > sortList.get(maxPos)){
        			maxPos = j;
        		}
  
        	}
        	returnList.add(list2.get(maxPos));
        	temp=sortList.get(maxPos);
        	sortList.set(maxPos, sortList.get(i));
        	sortList.set(i, temp);
        	
        }
        
//        System.out.println("pre sort "+list);
//        System.out.println("aft sort "+sortList);
        
        return returnList;
	}
	
	
}















































