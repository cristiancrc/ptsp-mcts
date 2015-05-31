package planners;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

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
	HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrixLava = new HashMap<>();//distance matrix from each waypoint to each other
	LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>();//the route resulted from the planner
	ArrayList<Path> m_plannedPath = new ArrayList<>();//path between waypoints ready to be displayed on-screen
	Graph m_graph;
	Game aGameCopy;
	
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
     * creates a distance matrix for a list of waypoints
	 * @param a list of waypoints
	 * @return a distance matrix
	 */
	public HashMap<Waypoint, HashMap<Waypoint, Double>> createDistanceMatrix(LinkedList<Waypoint> waypointList) {
    	
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
	 * @return a distance matrix
	 */
	public HashMap<Waypoint, HashMap<Waypoint, Double>> createDistanceMatrixLava(LinkedList<Waypoint> waypointList) {
    	
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
		                	distanceCost += 1.5*thisNode.euclideanDistanceTo(nextNode);
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
	 * displays a distance matrix in a humanly readable form
	 * @param distanceMatrix
	 */
	public void presentDistanceMatrix(HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix)
	{
		for (Waypoint wpFrom : distanceMatrix.keySet())    		
    	{
    		for (Waypoint wpTo : distanceMatrix.keySet()) //j = i for symmetric weights    			
    		{
    			if(wpTo.equals(wpFrom)) continue; //is 0
    			System.out.print("from:" + wpFrom.getName());
    			System.out.print(" to:" + wpTo.getName());
    			System.out.println(" =" + distanceMatrix.get(wpFrom).get(wpTo));
    		}
    	}
	}    
	
	/**
	 * show a list as consecutive waypoints and the total distance
	 * @param waypointList
	 */
    public void showList(LinkedList<Waypoint> waypointList)
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
     */
	public double getPathDistance(LinkedList<Waypoint> waypointList) {
		long timeIn = System.currentTimeMillis();
		double pathCost = 0;		
		for(int i = 1; i < waypointList.size(); i++)
   		{
			pathCost += distanceMatrix.get(waypointList.get(i-1)).get(waypointList.get(i));
   		}
		long timeOut = System.currentTimeMillis();
//		System.out.println(" Time spent inside getPathDistance: " + (timeOut - timeIn) + " ms.");
		return pathCost;
	}
	
    /**
     * calculate the total path distance of a list of waypoints
     * look-up in distanceMatrix 
     * @param waypointList
     * @return
     */
	public double getPathDistanceLava(LinkedList<Waypoint> waypointList) {
		long timeIn = System.currentTimeMillis();
		double pathCost = 0;		
		for(int i = 1; i < waypointList.size(); i++)
   		{
			pathCost += distanceMatrixLava.get(waypointList.get(i-1)).get(waypointList.get(i));
   		}
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside getPathDistanceLava: " + (timeOut - timeIn) + " ms.");
		return pathCost;
	}
	
	/**
	 * calculate the total path distance of a list of waypoints
	 * walk from node to node
	 * @param waypointList
	 * @param b0
	 * @return
	 */
	public double getPathDistanceWalk(LinkedList<Waypoint> waypointList, double b0) {
		long timeIn = System.currentTimeMillis();
		double pathCost = 0;
		for(int i = 1; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i-1);
			Waypoint wpEnd = waypointList.get(i);
			Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			Node nodeTo = m_graph.getClosestNodeTo(wpEnd.s.x, wpEnd.s.y, false);	
			Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());
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
                if(tempShip.isOnLava()) 
                {
                	distanceCost += b0*thisNode.euclideanDistanceTo(nextNode);
                } else 
                {
                    distanceCost += thisNode.euclideanDistanceTo(nextNode);
                }
            }
			pathCost += distanceCost;
   		}
		
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside getPathDistanceWalk: " + (timeOut - timeIn) + " ms.");
		return pathCost;
	}
	
	/**
	 * directness is a measure of how much a path deviates from a straight line
	 * so is the ratio of the path distance and the euclidean distance
	 * @param waypointList
	 * @return distance ratio
	 */
	public double getPathDirectness(LinkedList<Waypoint> waypointList) {
		long timeIn = System.currentTimeMillis();
		double distPath = 0;
		double distEuclidean = 0;
		for(int i = 1; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i-1);
			Waypoint wpEnd = waypointList.get(i);
			Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			Node nodeTo = m_graph.getClosestNodeTo(wpEnd.s.x, wpEnd.s.y, false);
			
			distPath += distanceMatrix.get(wpFrom).get(wpEnd);
			distEuclidean += nodeFrom.euclideanDistanceTo(nodeTo);
   		}
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside getPathDirectness: " + (timeOut - timeIn) + " ms.");
		return distPath/distEuclidean;
	}
	
	/**
	 * TODO 0 text and implementation
	 * @param waypointList
	 * @return
	 */
	private double getPathCostAngleChange(LinkedList<Waypoint> waypointList) {
		long timeIn = System.currentTimeMillis();
		Vector2d shipInit = aGameCopy.getShip().d;	
		System.out.println("ship initial vector : " + shipInit);
		//TODO -1 this raw approach does not represent the actual angle that the ship will have!
		// a node by node walk approach is still better!
		Vector2d v0v1 = new Vector2d(waypointList.get(1).s.x - waypointList.get(0).s.x, waypointList.get(1).s.y - waypointList.get(0).s.y);
		System.out.println("v0v1 vector : " + v0v1);
		
		double pathCost = 0;
		Vector2d entryVector = null;
		Vector2d exitVector = v0v1;
		for(int i = 1; i < waypointList.size(); i++)
   		{
			Waypoint wpFrom = waypointList.get(i-1);
			Waypoint wpEnd = waypointList.get(i);
			Node nodeFrom = m_graph.getClosestNodeTo(wpFrom.s.x, wpFrom.s.y, false);
			Node nodeTo = m_graph.getClosestNodeTo(wpEnd.s.x, wpEnd.s.y, false);			
			Path aPath = m_graph.getPath(nodeFrom.id(), nodeTo.id());		
												
			Node firstNode = m_graph.getNode(1);
			exitVector = new Vector2d( firstNode.x() - nodeFrom.x(), firstNode.y() - nodeFrom.y());
			
			double dotVector = entryVector.dot(exitVector);
			
			Node lastNode = m_graph.getNode(aPath.m_points.get(aPath.m_points.size()-2));//size-1 is last, size-2 is next to last
			entryVector = new Vector2d(nodeTo.x() - lastNode.x(), nodeTo.y() - lastNode.y() );

			
			System.out.println("entry vector : " + entryVector);					
   		}
		
		long timeOut = System.currentTimeMillis();
		System.out.println(" Time spent inside getPathAngleChange: " + (timeOut - timeIn) + " ms.");
		return 0.;
	}
	
	/**
	 * cost for a route is composed of:
	 *  distance cost
	 *  indirectness, how much the shortest path deviates from a straight line
	 *  change in angle between entering and leaving w[i+1]
	 * @param waypointList
	 * @return total cost
	 * TODO 0 - implement equation
	 */
	public double getPathCost(LinkedList<Waypoint> waypointList, double b1, double b2, double b3)
	{
		double costDistanceWalk = 0;		
		costDistanceWalk = getPathDistanceWalk(waypointList, 1.5);
		System.out.println("distance lava: " + costDistanceWalk);
		
		double costDistance = 0;		
		costDistance = getPathDistance(waypointList);
		System.out.println("distance matrix: " + costDistance);
		
		double costDistanceLava = 0;
		costDistanceLava = getPathDistanceLava(waypointList);
		System.out.println("distance matrix lava: " + costDistanceLava);		
		
		double costDirectness = 0;
		costDirectness = getPathDirectness(waypointList);
		System.out.println("directness: " + costDirectness);
		
		double costAngleChange = 0;
		costAngleChange = getPathCostAngleChange(waypointList);
		System.out.println("angle change: " + costAngleChange);

		double totalCost = Math.pow(costDistance, b1) + b2 *costDirectness + b3*costAngleChange;
		return totalCost;
	}
    
    

	public ArrayList<Path> getPlannedPath() {
		return m_plannedPath;
	}
	public LinkedList<Waypoint> getOrderedWaypoints() {
		return m_orderedWaypoints;
	}
    
	
}
