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

public class Agent {

	public ArrayList<Double> Position = new ArrayList<Double>();
	double MaxPos = 1.0; // boundaries of search space for particle, ie max min values for weights
	double MinPos = -1.0;
	double fitness;
	double novelty = -1;
	
	public Agent (int d){
		for(int i=0;i<d;i++){
			double startPos = (double) (Math.random() *( MaxPos - MinPos) ) + MinPos;
			this.Position.add(startPos);
		}
		fitness = Double.NEGATIVE_INFINITY;
	}
	
}
