package controllers.mcts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import framework.core.Controller;
import framework.core.Game;

public class SearchTreeNode {
	public SearchTreeNode parent = null;//parent node	
	public int action = -1;//action performed in parent node that got us here
	public Game worldSate;//current world state	
	public int depth;
	
	public int visited = 0;
	public double score = Double.NEGATIVE_INFINITY;
	public double value = Double.POSITIVE_INFINITY;
	private double alpha = Double.NEGATIVE_INFINITY;
	private double beta = Double.POSITIVE_INFINITY;
	
	public SearchTreeNode[] children = new SearchTreeNode[Controller.NUM_ACTIONS]; 	
	public Random rnd = new Random();
    public static double epsilon = 1e-6;
    public static double egreedyEpsilon = 0.05;
    public static double sqrt2 = Math.sqrt(2);
	
	
	public SearchTreeNode(Game a_gameCopy, SearchTreeNode parent) 
	{
		this.worldSate = a_gameCopy;
		this.parent = parent;
		children = new SearchTreeNode[Controller.NUM_ACTIONS];
		if(parent != null)
		    depth = parent.depth+1;
		else
		    depth = 0;
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
	
	
	public void trySetScore(double actionScore) 
	{
		System.out.println(actionScore + " ? " + this.value);
		// TODO Auto-generated method stub
		if(actionScore < this.value) 
			{
			System.out.println(actionScore + " < " + this.value);
			this.value = actionScore;
			}
	}


	public boolean notFullyExpanded() 
	{
        for (SearchTreeNode aNode : children) 
        {
            if (aNode == null) 
            {            	
                return true;
            }
        }
        return false;
    }
	
	public SearchTreeNode expand() 
	{
        int bestAction = 0;
        double bestValue = -1;

        for (int i = 0; i < children.length; i++) 
        {
            double x = rnd.nextDouble();
            if (x > bestValue && children[i] == null)
            {
                bestAction = i;
                bestValue = x;
            }
        }
        Game nextState = worldSate.getCopy();
        nextState.getShip().update(action);

        SearchTreeNode child = new SearchTreeNode(nextState, this);
        children[bestAction] = child;
        return child;

    }
	
	public SearchTreeNode uct() {
		SearchTreeNode selectedNode = null;
        double bestValue = -Double.MAX_VALUE;
        for (SearchTreeNode child : this.children)
        {
            double hvVal = child.value;
            double childValue =  hvVal / (child.visited + epsilon);

            //TODO:why normalize?
            childValue = normalise(childValue, alpha, beta);

            double uctValue = childValue + sqrt2 * Math.sqrt(Math.log(this.visited + 1) / (child.visited + epsilon));

            // small sampleRandom numbers: break ties in unexpanded nodes
            uctValue = noise(uctValue, epsilon, this.rnd.nextDouble());     //break ties randomly

            // small sampleRandom numbers: break ties in unexpanded nodes
            if (uctValue > bestValue) {
                selectedNode = child;
                bestValue = uctValue;
            }
        }

        if (selectedNode == null)
        {
            throw new RuntimeException("Warning! returning null: " + bestValue + " : " + this.children.length);
        }

        return selectedNode;
    }
	
	public SearchTreeNode egreedy() {
		SearchTreeNode selected = null;

        if(rnd.nextDouble() < egreedyEpsilon)
        {
            //Choose randomly
            int selectedIdx = rnd.nextInt(children.length);
            selected = this.children[selectedIdx];

        }else{
            //pick the best Q.
            double bestValue = -Double.MAX_VALUE;
            for (SearchTreeNode child : this.children)
            {
                double hvVal = child.value;
                hvVal = noise(hvVal, epsilon, this.rnd.nextDouble());     //break ties randomly
                // small sampleRandom numbers: break ties in unexpanded nodes
                if (hvVal > bestValue) {
                    selected = child;
                    bestValue = hvVal;
                }
            }

        }


        if (selected == null)
        {
            throw new RuntimeException("Warning! returning null: " + this.children.length);
        }

        return selected;
    }	
	
	public SearchTreeNode random() {
		SearchTreeNode selected = null;

        //Choose randomly
        int selectedIdx = rnd.nextInt(children.length);
        selected = this.children[selectedIdx];

        if (selected == null)
        {
            throw new RuntimeException("Warning! returning null: " + this.children.length);
        }

        return selected;
    }		
	
	/////////////////////////////////////////////////////////////////////////////////////////////////
    //Normalizes a value between its MIN and MAX.
    public static double normalise(double a_value, double a_min, double a_max)
    {
        if(a_min < a_max)
            return (a_value - a_min)/(a_max - a_min);
        else    // if bounds are invalid, then return same value
            return a_value;
    }
    
    /**
     * Adds a small noise to the input value.
     * @param input value to be altered
     * @param epsilon relative amount the input will be altered
     * @param random random variable in range [0,1]
     * @return epsilon-random-altered input value
     */
    public static double noise(double input, double epsilon, double random)
    {
        if(input != -epsilon) {
            return (input + epsilon) * (1.0 + epsilon * (random - 0.5));
        }else {
            //System.out.format("Utils.tiebreaker(): WARNING: value equal to epsilon: %f\n",input);
            return (input + epsilon) * (1.0 + epsilon * (random - 0.5));
        }
    }
}
