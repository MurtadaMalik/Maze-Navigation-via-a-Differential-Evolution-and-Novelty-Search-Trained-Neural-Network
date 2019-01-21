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

public class DE extends optimise {

	ArrayList <Agent> Agents;
	double CR = 0.9; // crossover probability 0.9
	double F = 0.5; // differential weight 0.5

	public DE(){
		super();
	}
	
	public DE(ArrayList<Integer> networkStructure,ArrayList<Double> prevGenWeights, 
			double par , int evals) {
		super(networkStructure, prevGenWeights, par,evals);
		// TODO Auto-generated constructor stub
		
		this.Agents = new ArrayList <Agent> ();
		int numAgents =numConnections;
		if(numAgents<4){
			numAgents=4;
		}
		for(int i=0;i<numAgents;i++){ 
			this.Agents.add(new Agent (numConnections)); 
		}
		this.maxI=evals;
		
	}

	public void trainNetwork(){
		this.bestFitness = Double.NEGATIVE_INFINITY;
		this.bestViolationCount = Integer.MAX_VALUE;
		this.bestViolationSum = Double.POSITIVE_INFINITY;

		
		deleteBehave();
		
		network.setWeights(prevGenWeights);
		evaluateCurNetwork();
		Agents.get(0).fitness= currentOverallFitness;
		Agents.get(0).Position = new ArrayList<Double>();
		Agents.get(0).Position = weightToPosition(prevGenWeights);
		if(currentOverallFitness>bestFitness){ 
			updateBestFitness();
		}
		evals++;
		
		addBestValues();
		
		for(int i=1;i<Agents.size(); i++){	
			network.setWeights(positionToWeights(Agents.get(i).Position));
			evaluateCurNetwork();
			Agents.get(i).fitness= currentOverallFitness;
			if(currentOverallFitness>bestFitness){ 
				updateBestFitness();
			}
			evals++;
			addBestValues();
			
		}
		
		for(int k=evals;k<maxI;k=k+Agents.size()){ 
			if(k%10==0){
	//			System.out.println("iteration "+k+" of "+maxI);
			}
			
			deleteBehave();
			ArrayList<ArrayList<Double>> newPositions = new ArrayList<ArrayList<Double>> ();
			for(int i=0;i<Agents.size(); i++){	
				if(!stop){
					
					ArrayList<Integer> potentialAgentsIndx = new ArrayList<Integer> ();
					for(int j=0;j<Agents.size();j++){
						if(i!=j){
							potentialAgentsIndx.add(j);
						}
					}
					
					ArrayList <Agent> agents3 = new ArrayList <Agent> ();
					for(int j=0;j<3;j++){
						int tempIndx1 = (int)(Math.random()*potentialAgentsIndx.size());
						int tempIndx2 = potentialAgentsIndx.get(	tempIndx1	);
						agents3.add(Agents.get(tempIndx2));
						potentialAgentsIndx.remove(tempIndx1);
					}
					
					int R = (int)( Math.random()* Agents.get(0).Position.size());
					ArrayList<Double> newPos = new ArrayList<Double> ();
					
					for(int j=0;j<Agents.get(0).Position.size();j++){
						double r = Math.random();
						if(r<CR || j==R){
							double a = agents3.get(0).Position.get(j);
							double b = agents3.get(1).Position.get(j);
							double c = agents3.get(2).Position.get(j);
							double y = a + (F*(b-c));
							newPos.add(y);
						}
						else{
							newPos.add(Agents.get(i).Position.get(j));
						}
					}
					newPositions.add(newPos);
					network.setWeights(positionToWeights(newPos));
					evaluateCurNetwork();
					evals++;
										
				}
				if(evals>=maxI){
					stop=true;
				}
				
			}
			
			if(noveltySearch){

				calcNovelty();
				for(int i=0;i<Agents.size();i++){
					double nov = Agents.get(i).novelty;
					Agents.get(i).novelty = nov*noveltyDecay;
				}
			}
			
			for(int i=0;i<curGenObjectiveFitnesses.size();i++){
				if(noveltySearch){
					
					double normFitness = curGenNormalObjectiveFitnesses.get(i);
					double normNovelty = normalNoveltyScores.get(i);
			//		double normNovelty = noveltyScores.get(i);
					
					
					double combineFit = ((1-ro)*normFitness) + (ro*normNovelty);
					
					if(combineFit>=Agents.get(i).novelty){
						Agents.get(i).fitness= curGenObjectiveFitnesses.get(i);
						Agents.get(i).novelty= normalNoveltyScores.get(i);
						Agents.get(i).Position = new ArrayList<Double>();
						Agents.get(i).Position = newPositions.get(i);
					}
					
				}
				else{
					if(curGenObjectiveFitnesses.get(i)>=Agents.get(i).fitness){
						Agents.get(i).fitness= curGenObjectiveFitnesses.get(i);
						Agents.get(i).Position = new ArrayList<Double>();
						Agents.get(i).Position =  newPositions.get(i);;
					}
				}
//				System.out.println("curFit = "+curGenObjectiveFitnesses.get(i));
//				System.out.println("bestFit = "+bestFitness);
				if(curGenObjectiveFitnesses.get(i)>bestFitness){ 
					updateBestFitness(i);		
				}
				addBestValues();
			}
			
						
			if(k%50==0 ){
				System.out.println("e "+evals+", Best Fit "+bestFitness);
			}
			
		}
		deleteBehave();
		network.setWeights(optimumNetworkWeights); 
		evaluateCurNetwork();
		
//		System.out.println("Locations found");
//		for(int i=0;i<finalRobotXPos.size();i++){
//			System.out.println(finalRobotXPos.get(i)+", "+finalRobotYPos.get(i));
//		}
		System.out.println("Completed "+evals+" evals");
		System.out.println("Best path found");
		for(int i=0;i<bestMazePathX.size();i++){
			System.out.println(bestMazePathX.get(i)+", "+bestMazePathY.get(i)+", "+i);
		}
		if(bestMazePathD.size()>0){
			System.out.println("End distance "+bestMazePathD.get(bestMazePathD.size()-1));
		}

	}
	
	
}


















