package controllers.mctsLive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Random;

import framework.core.Controller;
import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Node;
import framework.utils.Navigator;
import framework.utils.Value;
import framework.utils.Vector2d;

public class SearchTreeNodeLive {
	static boolean verbose = false;
    public SearchTreeNodeLive parent = null;//parent node   
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
    
    public SearchTreeNodeLive[] children = new SearchTreeNodeLive[Controller.NUM_ACTIONS];
//    public LinkedList<SearchTreeNodeLive> children = new LinkedList<SearchTreeNodeLive>();
    public Random rnd = new Random();
    public static double epsilon = 1e-6;
    public static double egreedyEpsilon = 0.05;
    public static double sqrt2 = Math.sqrt(2);
    
    
    public SearchTreeNodeLive(Game a_gameCopy, SearchTreeNodeLive parent) 
    {
    	this.name = hashCode();
        this.worldSate = a_gameCopy;
        this.parent = parent;
        
        children = new SearchTreeNodeLive[Controller.NUM_ACTIONS];
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
    public boolean hasChildObject(SearchTreeNodeLive possibleChild)
    {
        if(Arrays.asList(children).contains(possibleChild)) return true;
        return false;
    }
    
    /**
     * checks if a node has this specific child, for method
     * @param possibleChild
     * @return
     */
    public boolean hasChild(SearchTreeNodeLive possibleChild)
    {
        for(SearchTreeNodeLive a_node : children)
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
    public SearchTreeNodeLive getChild(int action) {
        for(SearchTreeNodeLive a_node : children)
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
        for (SearchTreeNodeLive aNode : this.children) 
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
    public SearchTreeNodeLive expand() 
    {
        /*
        //expand children nodes sequentially
        int childCount = 0;
        for (SearchTreeNodeLive aNode : this.children) 
        {        
            if (aNode != null) 
            {
                childCount++;
            }
        }
        int bestAction = childCount;
        */
        
        //expand new children nodes randomly
        int bestAction = 0;
        while(this.getChild(bestAction) != null)
        {
            bestAction = rnd.nextInt(Controller.NUM_ACTIONS);
        }       
        
        
        if (verbose) System.out.println(" expand action : " + bestAction);
        Game nextState = worldSate.getCopy();
        for (int _ = 0; _ < DriveMCTSLive.macroActionsCount; _++)
        {
//          System.out.println("\nship at: " + nextState.getShip().ps + " , h : " + nextState.getShip().d);
            nextState.getShip().update(bestAction);
//            System.out.println("performed " + bestAction);    
        }        

        SearchTreeNodeLive child = new SearchTreeNodeLive(nextState, this);
        children[bestAction] = child;
        child.action = bestAction;
        return child;

    }
    
    /**
     * upper confidence tree
     * @return
     */
    public SearchTreeNodeLive uct() {
    	//uctVal = number of wins / number of simulations of this child + exploration parameter * sqrt( log (total simulations) / number of simulations of this child
        SearchTreeNodeLive selectedNode = null;
        double bestValue = Double.MAX_VALUE;
        double explorationParameter = sqrt2;
       
        for (SearchTreeNodeLive child : this.children)
        {
            double hvVal = child.value;
            double childValue =  hvVal / (child.visited + epsilon);

            //TO DO 9 why normalize? bounds change
            childValue = normalise(childValue, bounds[0], bounds[1]);

            double uctValue = childValue - explorationParameter * Math.sqrt(Math.log(this.visited + 1) / (child.visited + epsilon));

            // small sampleRandom numbers: break ties in unexpanded nodes
            uctValue = noise(uctValue, epsilon, this.rnd.nextDouble());     //break ties randomly

            if (uctValue < bestValue) {//minimizing a cost function
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
    
    public SearchTreeNodeLive egreedy() {
        SearchTreeNodeLive selected = null;

        if(rnd.nextDouble() < egreedyEpsilon)
        {
            //Choose randomly
            int selectedIdx = rnd.nextInt(children.length);
            selected = this.children[selectedIdx];

        }else{
            //pick the best Q.
            double bestValue = -Double.MAX_VALUE;
            for (SearchTreeNodeLive child : this.children)
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
    
    public SearchTreeNodeLive random() {
        SearchTreeNodeLive selected = null;

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
    
    /**
     * select a random action and apply it to the current state until one of the finishRollout conditions is met or the target is reached
     * TO DO 8 the node would be better in the evaluation function than in the finish rollout
     * @param aimedNode
     * @return
     */
    public double simulateTarget(Node aimedNode) 
    {       
        Game nextState = worldSate.getCopy();       
        int thisDepth = this.depth;
        Vector2d nextPosition = new Vector2d();
                
    	while (!finishRolloutTarget(nextState, thisDepth, aimedNode))
        {
            int action = rnd.nextInt(Controller.NUM_ACTIONS);
            for (int _ = 0; _ < DriveMCTSLive.macroActionsCount; _++)
            {
//              System.out.print(action);
                nextState.tick(action);
            }            
            thisDepth++;
            nextPosition = nextState.getShip().s;
            if(!DriveMCTSLive.possiblePosition.contains(nextPosition))
            {
                DriveMCTSLive.possiblePosition.add(nextPosition);
            }
        }
        
        Value newStateValue = DriveMCTSLive.evaluateShipPositionVisibleNode(nextState, DriveMCTSLive.aimedNode);        
        double localNewValue = newStateValue.value;

// TO DO 9 this is modified while it is being drawn, resulting in problems
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
        System.out.print(".");
        if(depth >= DriveMCTSLive.searchDepth)
        {
        	if (verbose) System.out.print("max depth reached " + depth + ", limit at " + DriveMCTSLive.searchDepth);
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
     * check if this state is an end state to finish the simulation
     * @param aState
     * @param depth
     * @param aimedNode
     * @return
     */
    private boolean finishRolloutTarget(Game aState, int depth, Node aimedNode)
    {
        //rollout end conditions        
    	if (verbose) System.out.print(".");//TO DO 1 basic debug mcts
        if(depth >= DriveMCTSLive.searchDepth)
        {
        	if (verbose) System.out.print("max depth reached " + depth + ", limit at " + DriveMCTSLive.searchDepth);
            return true;            
        }  
        //TO DO 9 combine with updated evaluate position and remove the aimed node target
        if(4 > aimedNode.euclideanDistanceTo(aState.getShip().ps.x, aState.getShip().ps.y))
        {           
        	if (verbose) System.out.print("target checkpoint reached");
            return true;
        }
        
        if(aState.isEnded())
        {
        	if (verbose) System.out.print("game is ended()");
            return true;
        }           
        return false;
    }
    
    public void backPropagate(SearchTreeNodeLive fromThisNodeUpwards, double positionValue)
    {   
        SearchTreeNodeLive nodeBubble = fromThisNodeUpwards;
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

        if (verbose) System.out.println("bounds : " + SearchTreeNodeLive.bounds[0] + " <> " + SearchTreeNodeLive.bounds[1]);
        for (int i = 0; i < children.length; i++) 
        {
        	if (verbose) 
        	{
        		System.out.println("\nat child " + i + " with data");
        		System.out.println("visited : " + children[i].visited);
        		System.out.println("score : " + children[i].score);
        		System.out.println("best child : " + children[i].bestPossible);         
        		System.out.println("value : " + children[i].value);
        	}
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
        System.out.println("bounds : " + SearchTreeNodeLive.bounds[0] + " <> " + SearchTreeNodeLive.bounds[1]);
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
        for(SearchTreeNodeLive aNode : this.children)
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
    public static String getFullIdentifier(SearchTreeNodeLive baseNode)
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
    public static int getTotalChildren(SearchTreeNodeLive baseNode)
    {
        int totalChildren = 0;
        for(SearchTreeNodeLive aNode : baseNode.children)
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
	 * TO DO 2 use this for macroing, current copy looks fine
	 * TO DO 9 reuse the tree - this was reported bad due to different depth, so consider copying the tree and 
	 * first updating it before continuing with "normal" search
	 */
    public static SearchTreeNodeLive copyTree(SearchTreeNodeLive nodeStartFrom) 
    {
        SearchTreeNodeLive newRoot = new SearchTreeNodeLive(nodeStartFrom.worldSate, nodeStartFrom);
        newRoot.worldSate = nodeStartFrom.worldSate;
        newRoot.parent = nodeStartFrom.parent;
        newRoot.action = nodeStartFrom.action;
        newRoot.score = nodeStartFrom.score;
        newRoot.visited = nodeStartFrom.visited;
        newRoot.value = nodeStartFrom.value;
        newRoot.depth = nodeStartFrom.depth-1;//otherwise it is increased
        newRoot.name = nodeStartFrom.name;//copy hash code
        
        if(nodeStartFrom.depth == 1) 
        {
        	newRoot.parent = null;
        	newRoot.action = -1;
    	}

        for(SearchTreeNodeLive aNode : nodeStartFrom.children)
        {        	
        	if(null != aNode)
        	{
        		newRoot.children[aNode.action] = SearchTreeNodeLive.copyTree(aNode);        	
        	}
        }
        return newRoot;
    }
}
