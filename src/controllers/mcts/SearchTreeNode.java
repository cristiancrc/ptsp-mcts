package controllers.mcts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import ontology.Types;

import controllers.sampleMCTS.Agent;
import core.game.StateObservation;

import framework.core.Controller;
import framework.core.Game;

public class SearchTreeNode {
	//state
	public int action = -1;//action performed that got us here
	public int visited = 0;
	public double score = Double.NEGATIVE_INFINITY;
	public double value = Double.POSITIVE_INFINITY;
	public SearchTreeNode parent = null;
	public SearchTreeNode[] children = new SearchTreeNode[Controller.NUM_ACTIONS]; 
	private int depth;
	private double alpha = Double.NEGATIVE_INFINITY;
	private double beta = Double.POSITIVE_INFINITY;
	public Random rnd = new Random();
	
	public SearchTreeNode(SearchTreeNode parent) 
	{
		this.parent = parent;
		children = new SearchTreeNode[Controller.NUM_ACTIONS];
		if(parent != null)
		    depth = parent.depth+1;
		else
		    depth = 0;
    }
	
	public void mctsSearch(long a_timeDue) {

        long timeSpent = 0;
        long timeSpentAverage = 0;
        long remaining;
        remaining = a_timeDue - System.currentTimeMillis();
        int numIters = 0;

        int remainingLimit = 5;
        while(remaining > 2*timeSpentAverage && remaining > remainingLimit){
            long timerStart = System.currentTimeMillis();
            SearchTreeNode selectedNode = selectNextNode();
            double delta = selectedNode.rollOut();
            bubbleUpScore(selectedNode, delta);

            numIters++;
            timeSpent +=  System.currentTimeMillis() - timerStart;

            timeSpentAverage  = timeSpent / numIters;
            remaining = System.currentTimeMillis() - timerStart;
            //System.out.println(elapsedTimerIteration.elapsedMillis() + " --> " + acumTimeTaken + " (" + remaining + ")");
        }
        //System.out.println("-- " + numIters + " -- ( " + avgTimeTaken + ")");
    }	
	
	 public double rollOut(Game aGameCopy)
	    {
		 	Game rollerState = (Game) aGameCopy.getCopy();
	        int thisDepth = this.depth;

	        while (!finishRollout(rollerState,thisDepth)) {

	            int action = rnd.nextInt(DriveMCTS.NUM_ACTIONS);
//	            rollerState.advance(action);
	            thisDepth++;
	        }

	        double delta = value(rollerState);

	        if(delta < alpha)
	            alpha = delta;

	        if(delta > beta)
	            beta = delta;

	        return delta;
	    }
	 
	 public boolean finishRollout(Game rollerState, int depth)
	    {
	        if(depth >= DriveMCTS.searchDepth)      //rollout end condition.
	            return true;

	        if(DriveMCTS.isGameOver(rollerState))               //end of game
	            return true;

	        return false;
	    }
	 
	 public double value(Game a_gameState) {

	        boolean gameOver = (DriveMCTS.isGameOver(a_gameState));
	        //TODO eval ship	        
	        double rawScore = DriveMCTS.evaluateShipPosition(a_gameState);


//	        if(gameOver && win == Types.WINNER.PLAYER_LOSES)
//	            rawScore += HUGE_NEGATIVE;
//
//	        if(gameOver && win == Types.WINNER.PLAYER_WINS)
//	            rawScore += HUGE_POSITIVE;

	        return rawScore;
	    }
	
	
	public SearchTreeNode selectNextNode() {

		SearchTreeNode currentNode = this;

        while (currentNode.depth < DriveMCTS.searchDepth) //!currentNode.state.isGameOver()
        {
            if (currentNode.notFullyExpanded()) {
                return currentNode.expand();

            } else {
            	//random node
            	SearchTreeNode nextNode = currentNode.children[rnd.nextInt(currentNode.children.length)];
//            	SearchTreeNode nextNode = currentNode.uct();
                //SearchTreeNode nextNode = currentNode.egreedy();
                currentNode = nextNode;
            }
        }

        return currentNode;
    }
	
	/**
	 * update visited count and value and pass value on to parent
	 * @param node
	 * @param value
	 */
	public void bubbleUpScore(SearchTreeNode node, double value)
    {
        while(node != null)
        {
            node.visited++;
            node.value += value;
            node = node.parent;
        }
    }
	
	/**
	 * checks if a node has this specific child, asList method
	 * @param possibleChild
	 * @return
	 */
	public boolean hasChildObject(SearchTreeNode possibleChild)
	{
		if(Arrays.asList(children).contains(possibleChild)) return true;
		return false;
	}
	
	/**
	 * checks if a node has this specific child, for method
	 * @param possibleChild
	 * @return
	 */
	public boolean hasChild(SearchTreeNode possibleChild)
	{
		for(SearchTreeNode a_node : children)
		{
			if(a_node.action == possibleChild.action)
				return true;
		}
		return false;
	}
	
	/**
	 * if the node has a child for this action, return that child
	 * @param action
	 * @return
	 */
	public SearchTreeNode getChild(int action) {
		for(SearchTreeNode a_node : children)
		{
			if(a_node.action == action)
				return a_node;
		}
		return null;
	}
	
	/**
	 * draw the tree starting at the current node up to a specified depth
	 * @param depth
	 */
	public void present(int depth)
	{
		System.out.print("\n" +this.hashCode() + "(p:" + (this.parent != null ? this.parent.hashCode() : "-") + ")" + " [V:" + this.value + "] [a:" + this.action + "] [s:" + this.score + "] [v:" + this.visited + "] [c:" + this.children.length+"]");
		if(this.children.length > 0)
		{
			depth++;
			for(SearchTreeNode aNode : this.children)
			{
				System.out.print("\n");
				for(int i = 0; i < depth; i++) System.out.print("\t");
				aNode.present(depth);
			}
		}
	}
	
	
	public void trySetScore(double actionScore) {
		System.out.println(actionScore + " ? " + this.value);
		// TODO Auto-generated method stub
		if(actionScore < this.value) 
			{
			System.out.println(actionScore + " < " + this.value);
			this.value = actionScore;
			}
	}
	

}
