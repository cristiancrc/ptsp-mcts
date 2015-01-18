package controllers.mcts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import framework.core.Controller;
import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Node;
import framework.utils.Navigator;
import framework.utils.Value;
import framework.utils.Vector2d;

public class SearchTreeNode {
	public SearchTreeNode parent = null;//parent node	
	public int action = -1;//action performed in parent node that got us here
	public Game worldSate;//current world state	
	public int depth;
	
	public int visited = 0;
	public double score = 0;
	public double value = 0;
	public double bestPossible = Double.MAX_VALUE;
	protected static double[] bounds = new double[]{Double.MAX_VALUE, -Double.MAX_VALUE};
	
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
	
	/**
	 * return a random child (?)
	 * why bother with that draw and not get just get a rand 0 to length 
	 * @return
	 */
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
        child.action = bestAction;
        return child;

    }
	
	/**
	 * upper confidence tree
	 * @return
	 */
	public SearchTreeNode uct() {
		SearchTreeNode selectedNode = null;
        double bestValue = Double.MAX_VALUE;
        for (SearchTreeNode child : this.children)
        {
            double hvVal = child.value;
            double childValue =  hvVal / (child.visited + epsilon);

            //TODO:why normalize? bounds change
            childValue = normalise(childValue, bounds[0], bounds[1]);

            double uctValue = childValue - sqrt2 * Math.sqrt(Math.log(this.visited + 1) / (child.visited + epsilon));

            // small sampleRandom numbers: break ties in unexpanded nodes
            uctValue = noise(uctValue, epsilon, this.rnd.nextDouble());     //break ties randomly

            if (uctValue < bestValue) {
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


    //TODO: consider using just the state without the aimed node (update evaluation fn)
	public double simulate(Node aimedNode) 
	{
		Game nextState = worldSate.getCopy();        
        int thisDepth = this.depth;

        while (!finishRollout(nextState, thisDepth, aimedNode)) 
        {
            int action = rnd.nextInt(Controller.NUM_ACTIONS);
            for (int _ = 0; _ < DriveMCTS.macroActionsCount; _++)
            {
            	nextState.getShip().update(action);	
            }            
            thisDepth++;
            
            Vector2d nextPosition = nextState.getShip().s;
       	 	if(!DriveMCTS.possiblePosition.contains(nextPosition))
            {
       	 		DriveMCTS.possiblePosition.add(nextPosition);	
            }
        }

        Value newStateValue = Navigator.evaluateShipPosition(nextState, DriveMCTS.aimedNode);
        double localNewValue = newStateValue.value;
        if(localNewValue < bounds[0])
        {
            bounds[0] = localNewValue;
        }

        if(localNewValue > bounds[1])
        {
            bounds[1] = localNewValue;
        }

        return localNewValue;		
	}
	
	public boolean finishRollout(Game aState, int depth, Node aimedNode)
    {
		//rollout end conditions
		System.out.print(".");
		
        if(depth >= DriveMCTS.searchDepth)
        {
        	System.out.println("max depth reached");
            return true;            
        }    	      
        
        //TODO: target reached? aimedNode.RADIUS?
        if(4 > aimedNode.euclideanDistanceTo(aState.getShip().ps.x, aState.getShip().ps.y))
    	{    		
        	System.out.println("target checkpoint reached");
    		return true;
    	}
        
        if(aState.isEnded())
        {
        	System.out.println("game is ended()");
        	return true;
        }
            

        return false;
    }
	
	public void backPropagate(SearchTreeNode fromThisNodeUpwards, double positionValue)
    {	
		SearchTreeNode nodeBubble = fromThisNodeUpwards;
		while(nodeBubble != null)
        {			
			nodeBubble.visited++;
			nodeBubble.score += positionValue;
			nodeBubble.value = nodeBubble.score / nodeBubble.visited;
			if(positionValue < nodeBubble.bestPossible)
			{
				nodeBubble.bestPossible = positionValue;
			}
			nodeBubble = nodeBubble.parent;
        }
    }

	/**
	 * return most visited node
	 * @return
	 */
	public int getActionRobustChild() {
        int selectedAction = -1;
        double highestVisited = Double.MIN_VALUE;
        boolean allEqual = true;
        double initVisit = -1;

        System.out.println("bounds : " + SearchTreeNode.bounds[0] + " <> " + SearchTreeNode.bounds[1]);
        for (int i = 0; i < children.length; i++) 
        {
        	System.out.println("\nat child " + i + " with data");
        	System.out.println("visited : " + children[i].visited);
        	System.out.println("score : " + children[i].score);
        	System.out.println("best child : " + children[i].bestPossible);        	
        	System.out.println("value : " + children[i].value);
        	
            if(children[i] != null)
            {
            	//check if all children have the same visited count
                if(-1 == initVisit)
                {
                	initVisit = children[i].visited;
                }                    
                else if(initVisit != children[i].visited)
                {
                    allEqual = false;
                }

                double childVisitedCount = children[i].visited;
                if (childVisitedCount > highestVisited)
                {
                    highestVisited = childVisitedCount;
                    selectedAction = children[i].action;
                }
            }
        }

        if (-1 == selectedAction)
        {
            System.err.println("invalid action");
            selectedAction = 0;
        }
        else if(allEqual)
        {        	
        	//if all have been visited the same number of times, pick a random one
        	selectedAction = rnd.nextInt(children.length-1);
        	//TODO: or pick best action?
//        	selectedAction = bestAction();
        }
        return selectedAction;
    }

	/**
	 * return node with smallest value
	 * @return
	 */
    public int getActionMinValue()
    {
        int selectedAction = -1;
        double bestValue = Double.MAX_VALUE;
//        System.out.println("bounds : " + SearchTreeNode.bounds[0] + " <> " + SearchTreeNode.bounds[1]);
        for (int i = 0; i < children.length; i++) 
        {
            if(null != children[i])
            {
//            	System.out.println("\nat child " + i + " with data");            	            
//            	System.out.println("visited : " + children[i].visited);
//            	System.out.println("score : " + children[i].score);
//            	System.out.println("best child : " + children[i].bestPossible);            	
//            	System.out.println("value : " + children[i].value);
            	
                if (children[i].value < bestValue) {
                    bestValue = children[i].value;
                    selectedAction = i;
                }
            }
        }

        if (selectedAction == -1)
        {
        	System.err.println("invalid action");
            selectedAction = 0;
        }

        return selectedAction;
    }	
    
    /**
	 * path where a child had the lowest score (local min bound)
	 * @return
	 */
    public int getActionSecureChild()
    {
        int selectedAction = -1;
        double bestValue = Double.MAX_VALUE;
        System.out.println("bounds : " + SearchTreeNode.bounds[0] + " <> " + SearchTreeNode.bounds[1]);
        for (int i = 0; i < children.length; i++) 
        {
            if(null != children[i])
            {
            	System.out.println("\nat child " + i + " with data");            	            
            	System.out.println("visited : " + children[i].visited);
            	System.out.println("score : " + children[i].score);
            	System.out.println("best child : " + children[i].bestPossible);            	
            	System.out.println("value : " + children[i].value);
            	
                if (children[i].bestPossible < bestValue) {
                    bestValue = children[i].bestPossible;
                    selectedAction = i;
                }
            }
        }

        if (selectedAction == -1)
        {
        	System.err.println("invalid action");
            selectedAction = 0;
        }

        return selectedAction;
    }	    
}
