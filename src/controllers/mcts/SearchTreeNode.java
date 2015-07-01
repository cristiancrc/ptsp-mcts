package controllers.mcts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Random;

import planners.Planner3Opt;

import framework.core.Controller;
import framework.core.FuelTank;
import framework.core.Game;
import framework.core.GameObject;
import framework.core.PTSPConstants;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.utils.Navigator;
import framework.utils.Value;
import framework.utils.Vector2d;

public class SearchTreeNode {
	static boolean verbose = DriveMCTS.verbose;
    public SearchTreeNode parent = null;//parent node   
    public int action = -1;//action performed in parent node that got us here
    public Game worldSate;//current world state 
    public int depth;
    public boolean fullyExpanded = false;
    public int name;
    
    public int visited = 0;
    public double score = 0;
    public double value = 0;
    public double bestPossible = Double.MAX_VALUE;
    protected static double[] bounds = new double[]{Double.MAX_VALUE, -Double.MAX_VALUE};
    
    public SearchTreeNode[] children = new SearchTreeNode[Controller.NUM_ACTIONS];
    public Random rnd = new Random();
    public static double epsilon = 1e-6;
    public static double egreedyEpsilon = 0.05;
    
    //state evaluation data
//			+ 	DriveMCTS.w_wpDistance * distanceToNextWaypoint +
//			+   DriveMCTS.w_wpFuelOutOfRoute * collectedFuelOutOfRoute +
	private int collectedWpRoute;
	private int fuelConsumed;
    private double damageFromLava; //store damage from lava in tree
    private double damageFromCollisions; //store damage from collisions in tree
        
    public SearchTreeNode(Game a_gameCopy, SearchTreeNode parent) 
    {
    	this.name = hashCode();
        this.worldSate = a_gameCopy;
        this.parent = parent;
        
        
        children = new SearchTreeNode[Controller.NUM_ACTIONS];
        if(parent != null)
        {
            this.depth = parent.depth+1;
            this.damageFromLava = parent.damageFromLava;
            this.damageFromCollisions = parent.damageFromCollisions;
        }
        else
        {
            this.depth = 0;
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
            if(a_node != null && a_node.action == action)
                return a_node;
        }
        return null;
    }      
    
    /**
     * update the score of a node only if it is HIGHER than the current
     * @param actionScore
     */
    public void trySetScore(double actionScore) 
    {
    	if (verbose) System.out.println(actionScore + " ? " + this.value);
        if(actionScore < this.value) 
        {
        	if (verbose) System.out.println(actionScore + " < " + this.value);
            this.value = actionScore;
        }
    }

    /**
     * fast check if all the children nodes have been visited
     * @return
     */
    public boolean isFullyExpanded() 
    {    	
//        System.out.println("\nfully expanded fast check on " + this.getIdentifier());        
        for (SearchTreeNode aNode : this.children) 
        {        
            if (aNode == null) 
            {
                return false;
            }
        }
        return true;
    }
        
    /**
     * return a random child from the unexpanded ones
     * @return
     */
    public SearchTreeNode expand(boolean expandRandom) 
    {
        int bestAction = 0;
        if(expandRandom)
        {
            //expand new children nodes randomly        
            while(this.getChild(bestAction) != null)
            {
                bestAction = rnd.nextInt(Controller.NUM_ACTIONS);
            }                       	
        }
        else
        {
            //expand children nodes sequentially
            int childCount = 0;
            for (SearchTreeNode aNode : this.children) 
            {        
                if (aNode != null) 
                {
                    childCount++;
                }
            }
            bestAction = childCount;                    	
        }
        
        if (verbose) System.out.println(" expand action : " + bestAction);
        Game nextState = worldSate.getCopy();
        for (int _ = 0; _ < DriveMCTS.macroActionsCount; _++)
        {
//          System.out.println("\n ship at: " + nextState.getShip().ps + " , h : " + nextState.getShip().d);
            nextState.getShip().update(bestAction);
//            System.out.println("performed " + bestAction);    
        }        

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
    	//uctVal = number of wins / number of simulations of this child + exploration parameter * sqrt( log (total simulations) / number of simulations of this child
        SearchTreeNode selectedNode = null;
        double bestValue = Double.MAX_VALUE;
       
        for (SearchTreeNode child : this.children)
        {
            double hvVal = child.value;
            double childValue =  hvVal / (child.visited + epsilon);

            childValue = normalise(childValue, bounds[0], bounds[1]);

            double uctValue = childValue - DriveMCTS.ucb1Exploration * Math.sqrt(Math.log(this.visited + 1) / (child.visited + epsilon));

            // small sampleRandom numbers: break ties in unexpanded nodes
            uctValue = noise(uctValue, epsilon, this.rnd.nextDouble());     //break ties randomly

            if (uctValue < bestValue) 
            {//minimizing a cost function
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
            int aRandChild = rnd.nextInt(children.length);
            selected = this.children[aRandChild];
        }
        else
        {
            //pick the best Q.
            double bestValue = -Double.MAX_VALUE;
            for (SearchTreeNode child : this.children)
            {
                double hvVal = child.value;
                hvVal = noise(hvVal, epsilon, this.rnd.nextDouble());     //break ties randomly
                // small sampleRandom numbers: break ties in unexpanded nodes
                if (hvVal > bestValue)
                {
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

    public static double normalise(double a_value, double a_min, double a_max)
    {
        if(a_min < a_max)
        {    //Normalizes a value between its MIN and MAX.
            return (a_value - a_min)/(a_max - a_min);
        }
        else 
        {    // if bounds are invalid, then return same value
            return a_value;
        }
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

    /**
     * select a random action and apply it to the current state until one of the finishRollout conditions is met
     * @param ordered waypoints
     * @return
     * TODO 9 consider using dynamic depth
     */
    public double simulate(LinkedList<GameObject> orderedWaypoints, Graph aGraph) 
    {       
        Game nextState = worldSate.getCopy();       
        int thisDepth = this.depth;
        Vector2d nextPosition = new Vector2d();
                
        while (!finishRollout(nextState, thisDepth)) 
        {
            int action = rnd.nextInt(Controller.NUM_ACTIONS);
            nextState.getShip().update(action); 
            for (int _ = 1; _ < DriveMCTS.macroActionsCount; _++)
            {
//              System.out.print(action);
                nextState.getShip().update(action); 
            }            
            thisDepth++;
            nextPosition = nextState.getShip().s;
            if(!DriveMCTS.possiblePosition.contains(nextPosition))
            {
                DriveMCTS.possiblePosition.add(nextPosition);
            }
            
        }
        
        double localNewValue = evaluateShipPosition(nextState, orderedWaypoints, aGraph);        

//        nextPosition = nextState.getShip().s;
//        DriveMCTSLive.possiblePositionScore.putIfAbsent(nextPosition, localNewValue);
        
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
        
    /**
     * check if this state is an end state to finish the simulation
     * @param aState
     * @param depth
     * @return
     */
    public boolean finishRollout(Game aState, int depth)
    {
        //rollout end conditions        
        if (verbose) System.out.print(".");
        if(depth >= DriveMCTS.searchDepth)
        {
        	if (verbose) System.out.print("max depth reached " + depth + ", limit at " + DriveMCTS.searchDepth);
            return true;            
        }  
        
        if(aState.isEnded())
        {
        	if (verbose) System.out.print("game is ended()");
            return true;
        }           
        return false;
    }
    
    /**
     * bubble up score
     * @param fromThisNodeUpwards
     * @param positionValue
     */
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
     * computes a score / cost for getting from the current position to the aimedNode
     * @param aGameState
     * @return score
     */
    public double evaluateShipPositionBasic(Game aGameState, LinkedList<GameObject> orderedWaypoints, Graph aGraph) 
    {
    	Vector2d nextPosition = aGameState.getShip().s;
        Vector2d potentialDirection = aGameState.getShip().d;

    	GameObject target = null;
    	for(GameObject way : orderedWaypoints)
    	{
    		if(!isCollected(way))
    		{
    			target = way;
    			break;
    		}
    	}
    	Vector2d targetPosition = target.s;    		
    	Node nextNode = aGraph.getClosestNodeTo(targetPosition.x, targetPosition.y);
    	Vector2d nextNodeV = new Vector2d(nextNode.x(),nextNode.y());
        nextNodeV.subtract(nextPosition);
        nextNodeV.normalise();   //This is a unit vector from my position pointing towards the next node to go to.
        double dot = potentialDirection.dot(nextNodeV);  //Dot product between this vector and where the ship is facing to.
		double dist = nextNode.euclideanDistanceTo(nextPosition.x, nextPosition.y);
		if(verbose) System.out.println("\n dist: " + dist);
		if(verbose) System.out.println("dot : " + dot);
		double score = dist - 30* dot;
		return score;
    }
    /**
     * computes a score / cost for getting for a game state and a planned list
     * @param aGameState
     * @return score
     */
    public double evaluateShipPosition(Game aGameState, LinkedList<GameObject> orderedWaypoints, Graph aGraph) 
    {
    	//get number of waypoints and fueltanks collected on route
    	int collectedWpRoute = -1;
    	int collectedWp = 0;
    	int collectedFuelRoute = -1;
    	int collectedFuel = 0;
    	int nextTargetIndex = -1;
    	int wayCounter = 0;
    	for (GameObject way : orderedWaypoints) 
    	{    		
    		if(way instanceof Waypoint)
    		{
    			if (((Waypoint) way).isCollected())
    			{
    				collectedWp++;
				}
    			else
    			{
    				if( -1 == nextTargetIndex)
    				{
    					nextTargetIndex = wayCounter;
    				}
    				if( -1 == collectedWpRoute)
    				{
        				collectedWpRoute = collectedWp;
        			}    			
    			}
    			
    		}
    		else if(way instanceof FuelTank)
    		{
    			if (((FuelTank) way).isCollected())
    			{
    				collectedFuel++;
				}
    			else
    			{
    				if( -1 == nextTargetIndex)
    				{
    					nextTargetIndex = wayCounter;
    				}
    				if( -1 == collectedFuelRoute)
        			{
        				collectedFuelRoute = collectedFuel;
        			}   			
    			}    			
    		}
    		wayCounter++;
    	}
    	//get fuel collected out of route
    	int collectedFuelOutOfRoute = aGameState.getFuelTanksCollected() - collectedFuelRoute;
    	
    	//get number of wp collected out of route
    	int collectedWpOutOfRoute = aGameState.getWaypointsVisited()- collectedWpRoute;
    	
    	//get distance to next waypoint
    	Node shipNode = aGraph.getClosestNodeTo(aGameState.getShip().s.x, aGameState.getShip().s.y);
    	Node targetWpNode = aGraph.getClosestNodeTo(orderedWaypoints.get(nextTargetIndex).s.x, orderedWaypoints.get(nextTargetIndex).s.y);
    	double distanceToNextWaypoint = aGraph.getPath(targetWpNode.id(), shipNode.id()).m_cost;
    	
    	//get damage from collisions
    	int damageFromCollisions = 0;
       	if ( aGameState.getShip().getDamage() > this.worldSate.getShip().getDamage() 
       		&& aGameState.getShip().getCollLastStep())
       	{
       		damageFromCollisions += aGameState.getShip().getDamage() - this.worldSate.getShip().getDamage();
       	}

//		System.out.println("wp on route " + collectedWpRoute);
//		System.out.println("wp off route " + collectedWpOutOfRoute);
//		System.out.println("fuel on route " + collectedFuelRoute);
//		System.out.println("fuel off route " + collectedFuelOutOfRoute);
//		System.out.println("wp distance " + distanceToNextWaypoint);
//		System.out.println("fuel consumed " + (PTSPConstants.INITIAL_FUEL - aGameState.getShip().getRemainingFuel()));
//		System.out.println("damage taken " + (aGameState.getShip().getDamage()));
//		System.out.println("damage collisions " + damageFromCollisions);//		

    	//final score
    	double positionValue = 		DriveMCTS.w_wpCollectedRoute * collectedWpRoute +
    							+ 	DriveMCTS.search_wpCollectedOutOfRoute * collectedWpOutOfRoute +
    							+ 	DriveMCTS.w_wpDistance * distanceToNextWaypoint +
    							+ 	DriveMCTS.w_fuelConsumed * (PTSPConstants.INITIAL_FUEL - aGameState.getShip().getRemainingFuel()) +
    							+	DriveMCTS.w_damageIncurred * (aGameState.getShip().getDamage()) +
    							+   DriveMCTS.w_wpFuelOutOfRoute * collectedFuelOutOfRoute +
    							+ 	DriveMCTS.w_damageCollisions * damageFromCollisions;     	
		return positionValue;
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

        if (verbose) System.out.println("bounds : " + SearchTreeNode.bounds[0] + " <> " + SearchTreeNode.bounds[1]);
        for (int i = 0; i < children.length; i++) 
        {
            if(null != children[i])
            {
            	if (verbose) 
            	{
            		System.out.println("\nat child " + i + " with data");
            		System.out.println("visited : " + children[i].visited);
            		System.out.println("score : " + children[i].score);
            		System.out.println("best child : " + children[i].bestPossible);         
            		System.out.println("value : " + children[i].value);
            	}
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
            //or pick best action?
//          selectedAction = bestAction();
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
//        System.out.println("bounds : " + SearchTreeNodeLive.bounds[0] + " <> " + SearchTreeNodeLive.bounds[1]);
        for (int i = 0; i < children.length; i++) 
        {
            if(null != children[i])
            {
            	if (verbose) 
            	{
		              System.out.println("\nat child " + i + " with data");                           
		              System.out.println("visited : " + children[i].visited);
		              System.out.println("score : " + children[i].score);
		              System.out.println("best child : " + children[i].bestPossible);             
		              System.out.println("value : " + children[i].value);
            	}
		                
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
            	if (verbose) 
            	{
            		System.out.println("\nat child " + i + " with data");                           
	                System.out.println("visited : " + children[i].visited);
	                System.out.println("score : " + children[i].score);
	                System.out.println("best child : " + children[i].bestPossible);             
	                System.out.println("value : " + children[i].value);
            	}
                
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
    
    
    /**
     * draw the tree starting at the current node up to a specified depth
     * @param depth
     */
    public void present(int depth)
    {
    	if (verbose) System.out.print(this.getName() + " h<"+this.hashCode()+">" + " n<"+this.name+">" 
        		+ " {" + this.getIdentifier() + "} " 
        		+ "(p:" + (this.parent != null ? this.parent.getName() + ", " + this.parent.name : "-") + ")" 
        		+ " [V:" + this.value + "] [a:" + this.action + "] [s:" + this.score + "] [v:" + this.visited + "] [c:" + this.children.length+"]");
        depth++;
        for(SearchTreeNode aNode : this.children)
        {        	
        	if(null != aNode)
        	{
        		if (verbose) System.out.print("\n");
                for(int i = 0; i < depth; i++) System.out.print("\t ");
                aNode.present(depth);	
        	}
            
        }
    }
    public void present()
    {
    	present(0);
    }
    
    /**
     * humanly readable name of waypoint
     * @return string     
     */    
    public String getName()
    {
        return this.toString().substring(this.toString().indexOf("@")+1);       
    }
    
    public String getIdentifier()
    {
        return "d" + this.depth + ":a" + this.action;
    }
    
    /**
     * returns a string that shows how to get to this node from root (depth and action)
     * @param baseNode
     * @return
     */
    public static String getFullIdentifier(SearchTreeNode baseNode)
    {
        String parentIdentifier;
        if(baseNode.parent.action == -1)
        {//root node
            parentIdentifier = "root" + "> d" + baseNode.depth + ":a" + baseNode.action;
        } else {
            parentIdentifier = getFullIdentifier(baseNode.parent) + "> d" + baseNode.depth + ":a" + baseNode.action;    
        }       
        return parentIdentifier;        
    }
    
    /**
     * returns the number of children of this node and its children nodes
     * @param baseNode
     * @return
     */
    public static int getTotalChildren(SearchTreeNode baseNode)
    {
        int totalChildren = 0;
        for(SearchTreeNode aNode : baseNode.children)
        {
            if(null != aNode)
            {
                totalChildren += 1;//itself
                totalChildren += getTotalChildren(aNode);//its children nodes
            }
        }
        return totalChildren;
    }

	/**
	 * recursively returns a tree with all the children of the input node
	 * used to search while macroing another action 
	 * @param nodeStartFrom
	 * @return new node
	 * TODO 9 reuse the tree - this was reported bad due to different depth, so consider copying the tree and 
	 * first updating it before continuing with "normal" search
	 */
    public static SearchTreeNode copyTree(SearchTreeNode nodeStartFrom) 
    {
        SearchTreeNode newRoot = new SearchTreeNode(nodeStartFrom.worldSate, nodeStartFrom);
        newRoot.worldSate = nodeStartFrom.worldSate;
        newRoot.parent = nodeStartFrom.parent;
        newRoot.action = nodeStartFrom.action;
        newRoot.score = nodeStartFrom.score;
        newRoot.visited = nodeStartFrom.visited;
        newRoot.value = nodeStartFrom.value;
        newRoot.depth = nodeStartFrom.depth;//TODO 9 for copying a leaf this should be decreased
        newRoot.name = nodeStartFrom.name;//copy hash code
              
        for(SearchTreeNode aNode : nodeStartFrom.children)
        {        	
        	if(null != aNode)
        	{
        		newRoot.children[aNode.action] = SearchTreeNode.copyTree(aNode);        	
        	}
        }
        return newRoot;
    }
    
	
    /**
     * is collected for both waypoint and fuel tank
     * @param aGameObject
     * @return
     */
    public boolean isCollected(GameObject aGameObject)
    {
    	if(aGameObject instanceof Waypoint)
    	{
    		if(((Waypoint) aGameObject).isCollected())
    		{
    			return true;
    		}
    	}
    	if(aGameObject instanceof FuelTank)
    	{
    		if( ((FuelTank) aGameObject).isCollected())
    		{
    			return true;
    		}
    	}
    	return false;
    }
}
