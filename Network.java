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
import java.math.RoundingMode;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Scanner;




public class Network {
	int inputs;
	int outputs;
	int hidden;
	
	ArrayList <Integer> structure = new ArrayList <Integer>();
	ArrayList <ArrayList <Neuron>> network = new ArrayList <ArrayList <Neuron>>();

	ArrayList <Double> PreviousOutputs = new ArrayList <Double>();
	ArrayList <Double> CurrentOutputs = new ArrayList <Double>();
	int w;
	ArrayList <Integer> currentTopology = new ArrayList <Integer>();
	ArrayList <Double> currentWeights = new ArrayList <Double>();
	DecimalFormat df;
	
	boolean recurrent = true;
	
	public Network(int inputs, int outputs, ArrayList <Integer> structure){
		this.inputs = inputs;
		this.outputs = outputs;
		this.structure = structure;
		
		this.df = new DecimalFormat("#.###");
		this.df.setRoundingMode(RoundingMode.CEILING);
		
		if (recurrent){
			hidden=structure.get(0);
		}
		
		createNeurons();	
	//	System.out.println("inputs = "+inputs+", outputs = "+ outputs);
	}
	
	
	public ArrayList <Double> getNetworkOutput(ArrayList <Double> input){ // reads in an input and returns an array of outputs
		// method to calculate the output of the network for a given input
		PreviousOutputs = new ArrayList<Double>();
		PreviousOutputs = CurrentOutputs;
		
//		System.out.println("my PreviousOutputs "+PreviousOutputs);
//		System.out.println("my input "+input);

		ArrayList <Double> Outputs = new ArrayList <Double>();
		ArrayList <Double> neuronOutputs = new ArrayList <Double> ();
		
		for(int i=0;i<network.size();i++){ // for each layer of neurons 
			ArrayList <Double> unWeightedInputs = new ArrayList <Double>(); // outputs of previous layer before they are weighted
			
			if(i==0){ // if its the first hidden layer
				unWeightedInputs = input; 
				if(recurrent){
					for(int j=0;j<network.get(i).size();j++){
						unWeightedInputs.add(network.get(i).get(j).output);
					}
				}
//				System.out.println("number of network inputs "+input.size());
//				System.out.println("neurons in this layer "+network.get(i).size());
//				System.out.println("Unweighted inputs size "+unWeightedInputs.size());
			}
			
			else{
				unWeightedInputs = neuronOutputs;
			}

			
			neuronOutputs = new ArrayList <Double> (); // outputs from previous layer are removed 
			
			for(int j=0;j<network.get(i).size();j++){ // for each neuron in current layer
				network.get(i).get(j).setNetworkInputs(input);
				network.get(i).get(j).setInput(unWeightedInputs); // feeds unweighted input to current neuron
				// the inputs are weighted within the neurons getOutput() method
				double neuronOutput = network.get(i).get(j).getOutput();
				neuronOutputs.add(neuronOutput); // calculates output and adds it to array
			}

		}
		
		Outputs = neuronOutputs; // the final output is converted to binary signal, eg 0.9 0.5 0.4 -> 1 0 0
		
		this.CurrentOutputs =  new ArrayList <Double>();
		this.CurrentOutputs = Outputs;
		
//		System.out.println("my output "+Outputs);
		
		return Outputs;
	}
	

	public double [] activate(double [] input){
		ArrayList <Double> inputArray = listToArray(input);
//		for(int i=0;i<inputArray.size();i++){
//			System.out.println("Input "+i+" = "+inputArray.get(i));
//		}
		ArrayList <Double> outputs = getNetworkOutput(inputArray);
		return arrayToList(outputs);
	}
	
	public double[] arrayToList(ArrayList <Double> myList){
		double []newList = new double [myList.size()];
		
		for(int i=0;i<myList.size();i++){
			newList[i] = myList.get(i);
		}
		
		return newList;
	}
	
	public ArrayList <Double> listToArray(double[] myList){
		ArrayList <Double> tempArray = new ArrayList <Double> ();
		
		for(int i=0;i<myList.length;i++){
			tempArray.add(myList[i]);
		}
		
		return tempArray;
	}
	

	public void resetNeuronOutputs(){
		for(int i=0;i<network.size();i++){
			for(int j=0;j<network.get(i).size();j++){
				network.get(i).get(j).output=0.0;
			}
		}
	}

	
	public void createNeurons(){ // method to create neurons 
		if(!recurrent){
			for(int i =0;i<structure.size();i++){ // creates hidden neurons
				ArrayList <Neuron> layer = new ArrayList <Neuron>();
				
				if(i==0){
					for(int j =0;j<this.structure.get(i);j++){ // neurons in the first hidden layer
						layer.add(new Neuron(false, inputs, 0, outputs));
					}
				}
				else if (i!=structure.size()-1){
					for(int j =0;j<this.structure.get(i);j++){ // neurons in the first hidden layer
						layer.add(new Neuron(false, structure.get(i-1), 0, outputs));
					}
				}
				else{
					for(int j =0;j<this.structure.get(i);j++){
						layer.add(new Neuron(true, inputs,structure.get(i-1), outputs));
					}
				}
				network.add(layer); // layer of neurons added to network
			}
		}
		else{
			ArrayList <Neuron> layer = new ArrayList <Neuron>();
			for(int j =0;j<hidden;j++){ // neurons in the first hidden layer
				layer.add(new Neuron(false, inputs, hidden, outputs));
			}
			network.add(layer); // layer of neurons added to network
			layer = new ArrayList <Neuron>();
			
			for(int j =0;j<outputs;j++){
				layer.add(new Neuron(true, inputs, hidden, outputs));
			}
			network.add(layer); // layer of neurons added to network
		}
	}
	


	public ArrayList <Integer> getNetworkStructure(){
		ArrayList <Integer> npl = new ArrayList <Integer> ();
		
		for(int i=0;i<network.size();i++){
			npl.add(network.get(i).size());
		}
		
		return npl; // npl includes the output layer size
	}
	
	
	public ArrayList<Double> getWeights(){ // method to get weights from neurons so they may be passed to particles for optimisation
		ArrayList<Double> weights = new ArrayList<Double>();
		for(int i=0;i<network.size();i++){
			for(int j=0;j<network.get(i).size();j++){
				for(int k=0;k<network.get(i).get(j).weights.size();k++){
					weights.add(network.get(i).get(j).weights.get(k)); 
				}
			}
		}
		return weights;
	}
	
	public ArrayList<Integer> getTopology(){ 
		ArrayList<Integer> topology = new ArrayList<Integer>();
		for(int i=0;i<network.size();i++){
			for(int j=0;j<network.get(i).size();j++){
				for(int k=0;k<network.get(i).get(j).enabled.size();k++){
					topology.add(network.get(i).get(j).enabled.get(k)); 
				}
			}
		}
		return topology;
	}
	
	
	public void setWeights(ArrayList<Double> w){ // method to set network weights 
		// this method is used when evaluating a solution (set of weights) from a particle during optimisation
		int indx = 0;
		// method has similar structure to getWeights()
		for(int i=0;i<network.size();i++){
			for(int j=0;j<network.get(i).size();j++){
				for(int k=0;k<network.get(i).get(j).weights.size();k++){
					network.get(i).get(j).weights.set(k, w.get(indx)); 
					indx++;
				}
			}
		}
	}
	
	public void setNetworkTopology(ArrayList<Integer> networkTopology){
		int indx = 0;
		// method has similar structure to getWeights()
		for(int i=0;i<network.size();i++){
			for(int j=0;j<network.get(i).size();j++){
				for(int k=0;k<network.get(i).get(j).weights.size();k++){
					if(networkTopology.get(indx)==0){
						network.get(i).get(j).weights.set(k, 0.0); 
						network.get(i).get(j).setEnabled(k,0);
					}
					else{
						network.get(i).get(j).setEnabled(k,1);
					}
					indx++;
				}
			}
		}
	}
	
	public void addNeuron(ArrayList<Double> w, ArrayList<Integer> t, int layerNo){
		if(!recurrent){
			setWeights(w);
			setNetworkTopology(t);
			
			 ArrayList <Integer> newTopology = new  ArrayList <Integer>();
			
			for(int i=0;i<network.size();i++){
				if(i==layerNo){
					if(i==0){
						network.get(i).add(new Neuron(false, inputs, 0, outputs));
					}
					else{
						int prevLayerNeurons = network.get(i-1).size();
						network.get(i).add(new Neuron(false, prevLayerNeurons,0, outputs));
					}
				}
				if(i==layerNo+1){
					for(int j =0;j<network.get(i).size();j++){
						if(Math.random()>=0.5){
							network.get(i).get(j).weights.add((Math.random()*2.0)-1.0);
							network.get(i).get(j).enabled.add(1);
						}
						else{
							network.get(i).get(j).weights.add(0.0);
							network.get(i).get(j).enabled.add(0);
						}
					}
				}
			}
			
			for(int i=0;i<network.size();i++){
				for(int j=0;j<network.get(i).size();j++){
					 ArrayList <Integer> temp = network.get(i).get(j).getTopology();
					 for(int k=0;k<temp.size();k++){
						 newTopology.add(temp.get(k));
					 }
				}
			}
			
			currentTopology = new ArrayList <Integer>();
			currentTopology = newTopology;
			currentWeights = new ArrayList <Double>();
			currentWeights = getWeights();
		}
		
		else if(recurrent){
			setWeights(w);
			setNetworkTopology(t);
			
			for(int i=0;i< network.size();i++){
				for(int j=0;j<network.get(i).size();j++){
					if(Math.random()>=0.5){
						network.get(i).get(j).weights.add((Math.random()*2.0)-1.0);
						network.get(i).get(j).enabled.add(1);
					}
					else{
						network.get(i).get(j).weights.add(0.0);
						network.get(i).get(j).enabled.add(0);
					}
					network.get(i).get(j).hidden++;
				}
			}
			
			
			this.hidden++;
			network.get(0).add(new Neuron(false, inputs, hidden, outputs));		
		
			currentTopology = new ArrayList <Integer>();
			currentTopology = getTopology();
			currentWeights = new ArrayList <Double>();
			currentWeights = getWeights();
		}
	}
	
	public void addLayer(ArrayList<Double> w, ArrayList<Integer> t){
		setWeights(w);
		setNetworkTopology(t);

		int outputLayerIndx=network.size()-1;
		ArrayList <Neuron> outputLayer = network.get(outputLayerIndx);
		int numPrevLayerOutputs = network.get(outputLayerIndx-1).size();
		
		network.remove(outputLayerIndx);
		
		ArrayList <Neuron> newLayer = new ArrayList <Neuron>();
		for(int i=0;i<numPrevLayerOutputs;i++){
			newLayer.add(new Neuron(false, numPrevLayerOutputs, 0,outputs));
			for(int j=0;j<numPrevLayerOutputs;j++){
				double newW;
				int newE;
				if(i==j){
					newW=1.0;
					newE=1;
				}
				else{
					newW=0.0;
					newE=0;
				}
				newLayer.get(i).setW(j, newW); 
				newLayer.get(i).setEnabled(j, newE);
			}
		}
		network.add(newLayer);
		network.add(outputLayer);

		currentTopology = new ArrayList <Integer>();
		currentTopology = getTopology();
		currentWeights = new ArrayList <Double>();
		currentWeights = getWeights();
	}
	
	
	public ArrayList<Double> readFileNum(String path){ // method to read in file of numbers (integers or doubles)
		// used for reading in body_length etc of owls
		ArrayList<Double> contents = new ArrayList<Double>();
		
		try {
			Scanner s = null; 
			s = new Scanner(new BufferedReader(new FileReader(path))); 
			while (s.hasNext()) { 
				double n = s.nextDouble();
				contents.add(n); 
				}
			s.close();
			
		} catch (FileNotFoundException e) {
			
			e.printStackTrace();
		}
		return contents;
		
	}
	
	public ArrayList<String> readFileText(String path){ // method to read in file of strings eg, owl types
		ArrayList<String> contents = new ArrayList<String>();
		
		try {
			Scanner s = null; 
			s = new Scanner(new BufferedReader(new FileReader(path))); 
			while (s.hasNext()) { 
				contents.add(s.nextLine()); // next integer is added to randomList
				}
			s.close();
			
		} catch (FileNotFoundException e) {
			
			e.printStackTrace();
		}
		return contents;
		
	}
	
	
	
	public ArrayList <Double> toMaxBinary(ArrayList <Double> n){ 
		// finds max in array of real numbers and converts to binary where max -> 1 and others -> 0... eg 0.9 0.7 0.7 -> 1 0 0
		ArrayList <Double> binary = new ArrayList <Double> () ;
		int maxI=0;
		
		for(int i=0;i<n.size();i++){
			if(n.get(i)>n.get(maxI)){
				maxI=i;
			}
		}
		
		for(int i=0;i<n.size();i++){
			double val;
			if(i==maxI){
				val =1.0;
			}
			else{
				val =0.0;
			}
			
			binary.add(val);
		}
		
		return binary;
	}
	
	public ArrayList <Double> toBinary(ArrayList <Double> n){ // step function
		// converts real numbers between 0 and 1 to binary
		// used for step function
		ArrayList <Double> binary = new ArrayList <Double> () ;
			
		for(int i=0;i<n.size();i++){
			double val;
			if(n.get(i)>=0.5){
				val =1.0;
			}
			else{
				val =0.0;
			}
					
			binary.add(val);
		}
				
		return binary;
		
	}

	
	public void printWeights(){
		System.out.println("Printing Network Weights");
		for(int i=0;i<network.size();i++){
	//		System.out.println("Layer "+i);
			System.out.println("   |  ");
			for(int j=0;j<network.get(i).size();j++){
	//			System.out.println("Neuron "+j);
				for(int k=0;k<network.get(i).get(j).weights.size();k++){
					System.out.print(df.format(network.get(i).get(j).getW(k))+" ");
				}
	//			System.out.println();
				System.out.print("  +  ");
			}
			System.out.println();
		}
	}
	
	
	
	public int countEnabledConnections(){
		this.w=0;
		for(int i=0;i<network.size();i++){
			for(int j=0;j<network.get(i).size();j++){	
				for(int k=0;k<network.get(i).get(j).enabled.size();k++){
					if(network.get(i).get(j).enabled.get(k)==1){
						this.w++;
					}
				}
			}
		}
		return w;
	}
	


	

}





















