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

public class Neuron {
	
	ArrayList <Double> neuronInput;
	
	boolean outputNeuron;

	int numNetworkOutputs;
	int numNetworkInputs;
	int hidden;
	
	ArrayList <Double> networkInputs = new ArrayList <Double>(); // only used if neuron is in first hidden layer
	
	ArrayList <Double> weights = new ArrayList <Double>(); // array of weights for inputs to neuron
	ArrayList <Integer> enabled = new ArrayList <Integer>();
	
	double output=0;
	
	public Neuron(boolean outputNeuron, int numNetworkInputs, int hiddenNeurons, int numNetworkOut){
		this.outputNeuron = outputNeuron;
		this.numNetworkOutputs = numNetworkOut;
		this.numNetworkInputs = numNetworkInputs;
		this.hidden = hiddenNeurons;
		
		for(int i=0;i<numNetworkInputs;i++){
			this.networkInputs.add((Math.random()*2.0)-1.0); // initialised at random
		}
		
		if(!outputNeuron){
			for(int i=0;i<numNetworkInputs;i++){
				
				this.weights.add((Math.random()*2.0)-1.0);
				this.enabled.add(1);
			}
			
			for(int i=0;i<hidden;i++){
				this.weights.add((Math.random()*2.0)-1.0);
				this.enabled.add(1);
				
			}		
		}
		else{
			for(int i=0;i<hidden;i++){
				this.weights.add((Math.random()*2.0)-1.0);
				this.enabled.add(1);
			}
		}
		
	}
	
	
	public void setInput(ArrayList <Double> input){
		this.neuronInput = new ArrayList <Double>();
		this.neuronInput = input; // neuron reads in the unweighted inputs
	}
	
	public double getW(int i){
		return weights.get(i);
	}
	
	public void setW(int i, double v){
		weights.set(i,v);
	}
	
	public void setEnabled(int i, int v){
		enabled.set(i, v); 
	}
	
	public double getEnabled(int i){
		return enabled.get(i);
	}
	
	public ArrayList <Integer> getTopology(){
		return enabled; 
	}
	public void setTopology(ArrayList <Integer> topology){
		this.enabled = new ArrayList <Integer>();
		this.enabled = topology; 
	}
	
	public ArrayList <Double> getWeights(){
		return weights; 
	}
	public void setWeights(ArrayList <Double> weights){
		this.weights = new ArrayList <Double>();
		this.weights = weights; 
	}
	
	public void setNetworkInputs(ArrayList <Double> i){ // set input to network (for first hidden layer neurons use)
		this.networkInputs= i;
	}
	
	public double getOutput(){
		double v = 0.0; // v will be the sum of the weighted inputs 
		output = 0.0; // output of neuron
//		System.out.println("neuronInputSize = "+neuronInput.size());
//		System.out.println("enabledSize = "+enabled.size());
		
		for(int i=0;i<neuronInput.size();i++){
			if(enabled.get(i)==1){
				v+=(neuronInput.get(i)*weights.get(i)); // summing up the weighted inputs
			}
		}

		output = sigmoid(v);
		
	//	System.out.println("neuron output = "+output);
		
		return output;
	}
	
	public double sigmoid(double v){
		return 1.0/(1.0+(Math.exp(-v))) ;
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}
