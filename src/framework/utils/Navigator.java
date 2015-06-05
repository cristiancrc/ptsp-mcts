package framework.utils;

import java.util.ArrayList;

import framework.core.FuelTank;
import framework.core.Game;
import framework.core.Waypoint;
import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;

/**
 * a collection of methods common to several controllers
 */
public class Navigator {
    /**
     * Returns the last visible node in the path to the next waypoint
     * @return the node in the way to destination.
     * TODO 9 due to inertia ship can go behind a wall and end up not seeing the path at all: find a solution
     */
	public static Node getLastNodeVisible(Path a_Path, Game a_gameCopy, Graph m_graph)
    {
    	Node furthestNodeVisible =  m_graph.getNode(a_Path.m_points.get(0));
    	for(int i = 0; i < a_Path.m_points.size(); i++)
    	{
    		Node a_Node = m_graph.getNode(a_Path.m_points.get(i));
            Vector2d nextNodePos = new Vector2d(a_Node.x(),a_Node.y());
       		boolean isThereLineOfSight = a_gameCopy.getMap().LineOfSight(a_gameCopy.getShip().s, nextNodePos);
       		if(isThereLineOfSight)
       		{
       			furthestNodeVisible = a_Node;
       		}
    	}
    	return furthestNodeVisible;
    }
    
    /**
     * based on Diego Perez' code
     * computes a score / cost for getting from the current position to the aimedNode
     * @param a_gameCopy
     * @param aimedNode
     * @return
     */
    public static Value evaluateShipPosition(Game a_gameCopy, Node aimedNode) 
    {
    	Vector2d nextPosition = a_gameCopy.getShip().s;
		Vector2d potentialDirection = a_gameCopy.getShip().d;
		Vector2d aimedNodeV = new Vector2d(aimedNode.x(),aimedNode.y());
        
		aimedNodeV.subtract(nextPosition);
		aimedNodeV.normalise();   //This is a unit vector from my position pointing towards the next node to go to.
		double dot = potentialDirection.dot(aimedNodeV);  //Dot product between this vector and where the ship is facing to.
		//		Get the distance to the next node in the tree and update the total distance until the closest waypoint/fueltank
		double dist = aimedNode.euclideanDistanceTo(nextPosition.x, nextPosition.y);
		
		Value value = new Value();
		value.distance = dist;
		value.direction = dot;
		value.value = dist + dot;//TODO 0 this is a very bad heuristic
		return value;
	}

    /**
     * calculate paths from ship to each remaining object
     * @param a_gameCopy
     */
	public void calculateObjectPaths(Game a_gameCopy, ArrayList<Path> m_pathListWaypoints, ArrayList<Path> m_pathListFuelTanks)
    {
        Graph m_graph = new Graph(a_gameCopy);// the map graph
        Node m_shipNode = m_graph.getClosestNodeTo(a_gameCopy.getShip().s.x, a_gameCopy.getShip().s.y,true);        
    	//put all (remaining) waypoints paths here
        m_pathListWaypoints.clear();            
        for(Waypoint way: a_gameCopy.getWaypoints())
        {
            if(!way.isCollected())     //Only consider those not collected yet.
            {
            	m_pathListWaypoints.add(m_graph.getPath(m_shipNode.id(), m_graph.getClosestNodeTo(way.s.x, way.s.y,true).id()));
            }
        }
        
        m_pathListFuelTanks.clear();
        for(FuelTank tank: a_gameCopy.getFuelTanks())
        {
        	if(!tank.isCollected())
        		m_pathListFuelTanks.add(m_graph.getPath(m_shipNode.id(),m_graph.getClosestNodeTo(tank.s.x, tank.s.y,true).id()));
        }
    }
}


