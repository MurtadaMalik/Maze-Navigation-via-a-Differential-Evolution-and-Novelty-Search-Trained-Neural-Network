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

public class run {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		int numHiddenNeurons = 2;
		double noveltyDecay = 0.99;
		int numEvals = 40000;
		
		maze m = new maze();
		ArrayList<Integer> initialStructure = new  ArrayList<Integer>();
		initialStructure.add(numHiddenNeurons);
		initialStructure.add(m.outputSize);
		Network net =new Network(m.inputSize,m.outputSize,initialStructure); 
		
		DE de = new DE(initialStructure,net.getWeights(),
				noveltyDecay,numEvals);
		
		de.trainNetwork();
	}

}
