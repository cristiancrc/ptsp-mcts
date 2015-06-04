package planners;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import controllers.mcts.DriveMCTS;
import framework.core.Controller;
import framework.core.Game;
import framework.core.Ship;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;
import framework.utils.Vector2d;

/**
 *  abstract planner class
 *  @author Cristian
 *  @version 141128
 */	
public abstract class Planner {
	
	boolean verbose = false;
	HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance matrix from each waypoint to each other
	HashMap<Waypoint, HashMap<Waypoint, Double>> matrixCostLava = new HashMap<>();//cost (distance and lava) matrix from each waypoint to each other
	HashMap<Waypoint, HashMap<Waypoint, Double>> matrixCostDirectness = new HashMap<>();//directness matrix from each waypoint to each other
	HashMap<Waypoint, HashMap<Waypoint, HashMap<Waypoint, Double>>> matrixCostAngle = new HashMap<>();//directness matrix from each waypoint, through each other, to each other
	LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>();//the route resulted from the planner
	ArrayList<Path> m_plannedPath = new ArrayList<>();//path between waypoints ready to be displayed on-screen
	Graph m_graph;
	Game aGameCopy;
	static double b0 = 1.5;
	static double b1 = 1;
	static double b2 = 1;
	static double b3 = 1;
	
	static int callsCalc = 0;
	static int callsAngMatrix = 0;
	
    /**
     * store paths from one point to another, in the order resulted by a planner
     * @param a_gameCopy
     */
    public void calculateOrderedWaypointsPaths() 
    {
        //calculate paths from one waypoint to the next
        for (int i = 0; i < m_orderedWaypoints.size()-1; i++)
        {
        	Node nodeFrom = m_graph.getClosestNodeTo(m_orderedWaypoints.get(i).s.x, m_orderedWaypoints.get(i).s.y, false);//get node for the current waypoint
        	Node nodeTo = m_graph.getClosestNodeTo(m_orderedWaypoints.get(i+1).s.x, m_orderedWaypoints.get(i+1).s.y, false);//get node for the next waypoint
        	Path pathToNext = m_graph.getPath(nodeFrom.id(), nodeTo.id());//And get the path from one to the other
        	m_plannedPath.add(pathToNext);//store the path
        }
        
    }
    
    /**
     * create the 3d matrix containing angle changes between wapyoints
     * @param waypointList
     * @return 3d hash map, double
     * TODO 0 implement 
     */
    protected HashMap<Waypoint , HashMap<Waypoint, HashMap<Waypoint , Double>>> createAngleMatrix(LinkedList<Waypoint> waypointList)
    {
		long timeIn = System.currentTimeMillis();
    	HashMap<Waypoint, HashMap<Waypoint, HashMap<Waypoint, Double>>> matrixCostAngle = new HashMap<>();//directness matrix from each waypoint, through each other, to each other
		for(int i = 0; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i);
			Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			for (int j = 0; j < waypointList.size(); j++)
			{
				if(j == i) continue;
				Waypoint wpThrough = waypointList.get(j);
				Node nodeThrough = m_graph.getClosestNodeTo(wpThrough.s.x, wpThrough.s.y, false);
				for(int k =0; k < waypointList.size(); k++)
				{
					if(k == j) continue;
					if(k == i) continue;
					
					callsAngMatrix++;
					Waypoint wpTo = waypointList.get(k);				
					Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);
					System.out.println(wpFrom.getName() + " > " + wpThrough.getName() + " > " + wpTo.getName());
				}
			}			
   		}
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside createAngleMatrix: " + (timeOut - timeIn) + " ms.");
		return matrixCostAngle;
	}
    
	/**
	 * creates a matrix of directness between waypoints
	 * directness is a measure of how much a path deviates from a straight line
	 * so is the ratio of the path distance and the euclidean distance
	 * look-up in distanceMatrix
	 * @param waypointList
	 * @return directness matrix
	 */
	protected HashMap<Waypoint, HashMap<Waypoint, Double>> createDirectnessMatrix(LinkedList<Waypoint> waypointList) {
		long timeIn = System.currentTimeMillis();
		double distPath = 0;
		double distEuclidean = 0;
		HashMap<Waypoint, HashMap<Waypoint, Double>> matrixCostDirectness = new HashMap<>();//directness matrix from each waypoint to each other
		HashMap<Waypoint, Double> wpEndValue = new HashMap<>();//distance from -one- waypoint to each of the others
		for(int i = 0; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i);
			Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			wpEndValue.clear();		
			for (int j = 0; j < waypointList.size(); j++)
			{
				double ratio = 0;					
				Waypoint wpTo = waypointList.get(j);				
				Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);						
				
				if ( j == i){
					continue;
				}
				if ( j < i )
				{//retrieve
					ratio = matrixCostDirectness.get(wpTo).get(wpFrom);
				} else
				{//calculate
					distPath = distanceMatrix.get(wpFrom).get(wpTo);
					distEuclidean = nodeFrom.euclideanDistanceTo(nodeTo);					
					ratio = distPath/distEuclidean;									
				}
				wpEndValue.put(wpTo, ratio);
				matrixCostDirectness.put(wpFrom, (HashMap<Waypoint, Double>) wpEndValue.clone());
			}			
   		}
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside createDirectnessMatrix: " + (timeOut - timeIn) + " ms.");
		return matrixCostDirectness;
	}
    
    /**
     * creates a distance matrix for a list of waypoints
	 * @param a list of waypoints
	 * @return a distance matrix
	 * TODO 3 set to private and see if instances need cost not just distance
	 */
	protected HashMap<Waypoint, HashMap<Waypoint, Double>> createDistanceMatrix(LinkedList<Waypoint> waypointList) {
    	
		//compute distance matrix for waypoints
		HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance from -each- waypoint to each of the others
       	HashMap<Waypoint, Double> distanceList = new HashMap<>();//distance from -one- waypoint to each of the others
       		
    	for (int i = 0; i < waypointList.size(); i++)    		
    	{
    		Waypoint wpFrom = waypointList.get(i);
    		distanceList.clear();
    		
    		for (int j = 0; j < waypointList.size(); j++) 
			//j = i for symmetric weights , j = 0 for asymmetric (uphill downhill)
    		//we need full matrix for easy lookup, so j = 0, but we will avoid computing cost by retrieving previous result
    		{
    			Waypoint wpTo = waypointList.get(j);    			
    			if (i == j) continue; //distance to self is 0
    			double pathCost = 0;
    			//avoid computing the path cost for symmetric distances, and retrieve previous result
    			if (j < i )
    			{
					pathCost = distanceMatrix.get(wpTo).get(wpFrom);	
//    				System.out.println(" cost retrieved:" + pathCost);
    			} 
    			else
    			{
    				Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
    				Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);					
    				Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());
    	    		pathCost = aPath.m_cost;
//    	    		System.out.println(" calculated new path cost:" + pathCost);
    			}
	        	distanceList.put(wpTo, pathCost);	        	
    		}
    		distanceMatrix.put(wpFrom, (HashMap<Waypoint, Double>) distanceList.clone());
    	}
    	return distanceMatrix;
	}
	
    /**
     * creates a distance matrix for a list of waypoints, taking lava into account
	 * @param a list of waypoints
	 * @param lava multiplier
	 * @return a distance matrix
	 */
	private HashMap<Waypoint, HashMap<Waypoint, Double>> createDistanceMatrixLava(LinkedList<Waypoint> waypointList) {
    	
		//compute distance matrix for waypoints
		HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance from -each- waypoint to each of the others
       	HashMap<Waypoint, Double> distanceList = new HashMap<>();//distance from -one- waypoint to each of the others
       		
    	for (int i = 0; i < waypointList.size(); i++)    		
    	{
    		Waypoint wpFrom = waypointList.get(i);
    		distanceList.clear();
    		
    		for (int j = 0; j < waypointList.size(); j++) 
			//j = i for symmetric weights , j = 0 for asymmetric (uphill downhill)
    		//we need full matrix for easy lookup, so j = 0, but we will avoid computing cost by retrieving previous result
    		{
    			Waypoint wpTo = waypointList.get(j);    			
    			if (i == j) continue; //distance to self is 0
    			double pathCost = 0;
    			//avoid computing the path cost for symmetric distances, and retrieve previous result
    			if (j < i )
    			{
					pathCost = distanceMatrix.get(wpTo).get(wpFrom);	
//    				System.out.println(" cost retrieved:" + pathCost);
    			} 
    			else
    			{
    				Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
    				Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);					
    				Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());
    	    		pathCost = aPath.m_cost;//TODO 1 init cost
    	    		double distanceCost = 0;
    	    		for(int k = 0; k < aPath.m_points.size()-1; k++)
		            {
		                Node thisNode = m_graph.getNode(aPath.m_points.get(k));
		                Node nextNode = m_graph.getNode(aPath.m_points.get(k+1));
		                	                
		                Game tempGame = aGameCopy.getCopy();		                
		                Ship tempShip = tempGame.getShip();
		                tempShip.s.x = nextNode.x();
		                tempShip.s.y = nextNode.y();
		                tempGame.setShip(tempShip);
		                tempGame.tick(Controller.ACTION_NO_LEFT);//this tick is needed to register if the ship is above lava, using ACTION_NO_FRONT will not work for tempShip.isOnLava()
		                if(tempShip.isOnLava()) {
		                	distanceCost += b0*thisNode.euclideanDistanceTo(nextNode);
		                }
		                else {
		                	distanceCost += thisNode.euclideanDistanceTo(nextNode);
		                }		               		                  
		            }    	    		
//    	    		System.out.println("Calculate Path cost:  " + pathCost + " , lava cost : " + distanceCost);
    	    		pathCost = distanceCost;
    			}    			
	        	distanceList.put(wpTo, pathCost);	        	
    		}
    		distanceMatrix.put(wpFrom, (HashMap<Waypoint, Double>) distanceList.clone());
    	}
    	return distanceMatrix;
	}

	/**
	 * creates the distance matrix and the distance matrix with lava cost, both at the same time
	 * is usually faster than creating both of them separately
	 * @param waypointList
	 * @return array with two distance matrices
	 * TODO ~ incomplete implementation
	 */
	protected HashMap<Waypoint, HashMap<Waypoint, Double>>[] createDistanceMatrices(LinkedList<Waypoint> waypointList) {
    	
		HashMap<Waypoint, HashMap<Waypoint, Double>>[] result = new HashMap[2];
		
		//compute distance matrix for waypoints
		HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance from -each- waypoint to each of the others
		HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrixLava = new HashMap<>();//distance from -each- waypoint to each of the others
       	HashMap<Waypoint, Double> distanceList = new HashMap<>();//distance from -one- waypoint to each of the others
       	HashMap<Waypoint, Double> distanceListLava = new HashMap<>();//distance from -one- waypoint to each of the others
       		
    	for (int i = 0; i < waypointList.size(); i++)    		
    	{
    		Waypoint wpFrom = waypointList.get(i);
    		distanceList.clear();
    		distanceListLava.clear();
    		
    		for (int j = 0; j < waypointList.size(); j++) 
			//j = i for symmetric weights , j = 0 for asymmetric (uphill downhill)
    		//we need full matrix for easy lookup, so j = 0, but we will avoid computing cost by retrieving previous result
    		{
    			Waypoint wpTo = waypointList.get(j);    			
    			if (i == j) continue; //distance to self is 0
    			double pathCost = 0;
    			double lavaCost = 0;
    			//avoid computing the path cost for symmetric distances, and retrieve previous result
    			if (j < i )
    			{
					pathCost = distanceMatrix.get(wpTo).get(wpFrom);
					lavaCost = distanceMatrixLava.get(wpTo).get(wpFrom);
//    				System.out.println(" cost retrieved:" + pathCost);
    			} 
    			else
    			{
    				Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
    				Node nodeTo = m_graph.getClosestNodeTo(wpTo.s.x, wpTo.s.y, false);					
    				Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());
    	    		pathCost = aPath.m_cost;
    	    		double distanceCost = 0;
    	    		for(int k = 0; k < aPath.m_points.size()-1; k++)
		            {
		                Node thisNode = m_graph.getNode(aPath.m_points.get(k));
		                Node nextNode = m_graph.getNode(aPath.m_points.get(k+1));
		                	                
		                Game tempGame = aGameCopy.getCopy();		                
		                Ship tempShip = tempGame.getShip();
		                tempShip.s.x = nextNode.x();
		                tempShip.s.y = nextNode.y();
		                tempGame.setShip(tempShip);
		                tempGame.tick(Controller.ACTION_NO_LEFT);//this tick is needed to register if the ship is above lava, using ACTION_NO_FRONT will not work for tempShip.isOnLava()
		                if(tempShip.isOnLava()) {
		                	distanceCost += b0*thisNode.euclideanDistanceTo(nextNode);
		                }
		                else {
		                	distanceCost += thisNode.euclideanDistanceTo(nextNode);
		                }		               		                  
		            }    	    		
    	    		lavaCost = distanceCost;
    			}    			
	        	distanceList.put(wpTo, pathCost);
	        	distanceListLava.put(wpTo, lavaCost);
    		}
    		distanceMatrix.put(wpFrom, (HashMap<Waypoint, Double>) distanceList.clone());
    		distanceMatrixLava.put(wpFrom, (HashMap<Waypoint, Double>) distanceListLava.clone());
    	}
    	result[0] = distanceMatrix;
    	result[1] = distanceMatrixLava;
    	return result;
	}

	/**
	 * displays a matrix in a humanly readable form
	 * @param matrix
	 */
	protected void presentMatrix(HashMap<Waypoint, HashMap<Waypoint, Double>> matrix)
	{
		for (Waypoint wpFrom : matrix.keySet())    		
    	{
    		for (Waypoint wpTo : matrix.keySet()) //j = i for symmetric weights    			
    		{
    			if(wpTo.equals(wpFrom)) continue; //is 0
    			System.out.print("from:" + wpFrom.getName());
    			System.out.print(" to:" + wpTo.getName());
    			System.out.println(" =" + matrix.get(wpFrom).get(wpTo));
    		}
    	}
	}    
	
	/**
	 * show a list as consecutive waypoints
	 * @param waypointList
	 */
    protected void showList(LinkedList<Waypoint> waypointList)
    {
    	System.out.println("");
    	for (Waypoint way : waypointList)
    	{
    		System.out.print(way.getName() + " ");
    	}
    	System.out.println("");
    }
    
    /**
     * calculate the total path distance of a list of waypoints
     * look-up in distanceMatrix 
     * @param waypointList
     * @return
     * TODO 3 set to private and see if calls need cost not just distance 
     */
	protected double getPathDistance(LinkedList<Waypoint> waypointList) {

		double pathCost = 0;		
		for(int i = 1; i < waypointList.size(); i++)
   		{
			pathCost += distanceMatrix.get(waypointList.get(i-1)).get(waypointList.get(i));
   		}		
		return pathCost;
	}
	
    /**
     * calculate the total path distance of a list of waypoints
     * look-up in matrixCostLava 
     * @param waypointList
     * @return
     */
	protected double getPathDistanceLava(LinkedList<Waypoint> waypointList) {
		double pathCost = 0;		
		for(int i = 1; i < waypointList.size(); i++)
   		{			
			pathCost += matrixCostLava.get(waypointList.get(i-1)).get(waypointList.get(i));
   		}
		return pathCost;
	}
	
	/**
	 * directness is a measure of how much a path deviates from a straight line
	 * so is the ratio of the path distance and the euclidean distance
	 * look-up in matrixCostDirectness
	 * @param waypointList
	 * @return distance ratio
	 */
	private double getPathDirectness(LinkedList<Waypoint> waypointList) {
		double pathRatio = 0;
		for(int i = 1; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i-1);
			Waypoint wpTo = waypointList.get(i);			
			pathRatio += matrixCostDirectness.get(wpFrom).get(wpTo); 			
   		}
		return pathRatio;
	}
	
	/**
	 * returns total angle cost for a path
	 * look-up in matrixCostAngle
	 * @param waypointList
	 * @return
	 */
	private double getPathCostAngleChange(LinkedList<Waypoint> waypointList) {
		double angleCost = 0;
		for(int i = 1; i < waypointList.size()-1; i++)
   		{
			Waypoint wpFrom = waypointList.get(i-1);
			Waypoint wpThrough = waypointList.get(i);
			Waypoint wpTo = waypointList.get(i+1);
			angleCost += matrixCostAngle.get(wpFrom).get(wpThrough).get(wpTo); 			
   		}
		return angleCost;
	}
	
	/**
	 * calculates the cost of angle change for a list of waypoints
	 * @param waypointList
	 * @return total cost
	 * TODO 2 create 3d matrix with values
	 */
	private double calculateCostAngleChange(LinkedList<Waypoint> waypointList) {
		boolean verbose = DriveMCTS.verbose;
		long timeIn = System.currentTimeMillis();
		double pathCost = 0;
		
//		System.out.println("ship direction " + aGameCopy.getShip().d);
		Vector2d shipInit = aGameCopy.getShip().d;
//		System.out.println("way1 position " + waypointList.get(1).s);
//		System.out.println("ship position " + aGameCopy.getShip().ps);
		
		//initial ship heading and first waypoint
		//waypointList[0] is the ship position
		Waypoint wpFrom = waypointList.get(1);
		Waypoint wpEnd = waypointList.get(2);
		Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
		Node nodeTo = m_graph.getClosestNodeTo(wpEnd.s.x, wpEnd.s.y, false);
		Path pathIncoming = null;
		Path pathOutgoing = m_graph.getPath(nodeFrom.id(), nodeTo.id());
//		System.out.println("node 0 " + m_graph.getNode(pathOutgoing.m_points.get(0)).x() + " : " + m_graph.getNode(pathOutgoing.m_points.get(0)).y());
//		System.out.println("node 1 " + m_graph.getNode(pathOutgoing.m_points.get(1)).x() + " : " + m_graph.getNode(pathOutgoing.m_points.get(1)).y());
		Node firstNode = m_graph.getNode(pathOutgoing.m_points.get(1));
		Node lastNode;
		
		Vector2d entryVector = shipInit;
		Vector2d exitVector = new Vector2d(firstNode.x() - nodeFrom.x(), firstNode.y() - nodeFrom.y());
//		System.out.println("from way0 to way1 actual exit vector : " + exitVector);
		exitVector.normalise();
		double dotVector = -entryVector.dot(exitVector);
		
//		System.out.printf("node direction: %f\n", dotVector);
		dotVector = 0.;
		pathCost += dotVector;		
					
		for(int i = 1; i < waypointList.size()-1; i++)
   		{			
			callsCalc++;
			wpFrom = waypointList.get(i);
			wpEnd = waypointList.get(i+1);
			nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			nodeTo = m_graph.getClosestNodeTo(wpEnd.s.x, wpEnd.s.y, false);			
			pathIncoming = pathOutgoing;
			pathOutgoing = m_graph.getPath(nodeFrom.id(), nodeTo.id());
			
			lastNode = m_graph.getNode(pathIncoming.m_points.get(pathIncoming.m_points.size()-2));//size-1 is last, size-2 is next to last
			entryVector = new Vector2d(nodeFrom.x() - lastNode.x(), nodeFrom.y() - lastNode.y() );
			entryVector.normalise();

			firstNode = m_graph.getNode(pathOutgoing.m_points.get(1));
			exitVector = new Vector2d(firstNode.x() - nodeFrom.x(), firstNode.y() - nodeFrom.y());
			exitVector.normalise();

			dotVector = entryVector.dot(exitVector);
//			System.out.printf("\nw[%d]->w[%d] : entry(%f,%f) . exit(%f,%f) = dot(%f)", i, i+1, entryVector.x, entryVector.y, exitVector.x, exitVector.y, dotVector);
			pathCost += dotVector;						
   		}
		
		long timeOut = System.currentTimeMillis();
//		System.out.println("\nTime spent inside getPathAngleChange: " + (timeOut - timeIn) + " ms.");
		return pathCost;
	}
	
	/**
	 * cost for a route is composed of:
	 *  distance cost
	 *  indirectness, how much the shortest path deviates from a straight line
	 *  change in angle between entering and leaving w[i+1]
	 * @param waypointList
	 * @return total cost
	 */
	public double getPathCost(LinkedList<Waypoint> waypointList, double b1, double b2, double b3)
	{		
//		double costDistance = 0;		
//		costDistance = getPathDistance(waypointList);
//		System.out.println("distance matrix: " + costDistance);
		
		double costDistanceLava = 0;
		costDistanceLava = getPathDistanceLava(waypointList);
//		System.out.println("distance matrix lava: " + costDistanceLava);		
		
		double costDirectness = 0;
		costDirectness = getPathDirectness(waypointList);
//		System.out.println("directness: " + costDirectness);
		
		double costAngleChange = 0;
//		costAngleChange = getPathCostAngleChange(waypointList);		
//		System.out.println("angle change: " + costAngleChange);
		
		calculateCostAngleChange(waypointList);
		
		double totalCost = Math.pow(costDistanceLava, b1) + b2 *costDirectness + b3*costAngleChange;
		return totalCost;
	}	


	public double getPathCost(LinkedList<Waypoint> waypointList)
	{
		return getPathCost(waypointList, b1, b2, b3);		
	}        
	public ArrayList<Path> getPlannedPath() 
	{
		return m_plannedPath;
	}
	public LinkedList<Waypoint> getOrderedWaypoints() 
	{
		return m_orderedWaypoints;
	}
}
