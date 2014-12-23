package planners;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;

/**
 *  abstract planner class
 *  @author Cristian
 *  @version 141128
 */	
public abstract class Planner {
	
	boolean verbose = false;
	HashMap<Waypoint, HashMap<Waypoint, Double>> distanceMatrix = new HashMap<>();//distance matrix from each waypoint to each other
	LinkedList<Waypoint> m_orderedWaypoints = new LinkedList<>();//the route resulted from the planner
	ArrayList<Path> m_plannedPath = new ArrayList<>();//path between waypoints ready to be displayed on-screen
	Graph m_graph;
	
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
     * calculate the total Euclidean distance of a list of waypoints
     * @param aPath
     * @return
     */
	public double getPathDistance(LinkedList<Waypoint> aPath) {
		double pathCost = 0;		
		for(int i = 1; i < aPath.size(); i++)
   		{
//			System.out.println("\napath i-1 " + aPath.get(i-1).getName());
//			System.out.println("apath i " + aPath.get(i).getName());
//			System.out.println("distmatrix " + distanceMatrix.get(aPath.get(i-1)).get(aPath.get(i)));
			pathCost += distanceMatrix.get(aPath.get(i-1)).get(aPath.get(i));
   		}
		return pathCost;
	}    
    
    public ArrayList<Path> getPlannedPath() {
		return m_plannedPath;
	}
	public LinkedList<Waypoint> getOrderedWaypoints() {
		return m_orderedWaypoints;
	}
    
	
}
