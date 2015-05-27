package framework.utils;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import framework.graph.Graph;
import framework.graph.Node;
import framework.graph.Path;

/**
 * a collection of methods common to several controllers
 * @author Cristian
 * @version 141222
 *
 */
public class Painter {
	/**
	 * draw the node where the ship is going for 
	 * @param a_gr
	 * @param aimedNode
	 */
	public static void paintAimedNode(Graphics2D a_gr, Node aimedNode, Color color)
	{
		a_gr.setColor(color);
    	a_gr.fillOval(aimedNode.x(), aimedNode.y(), 5, 5);
	}
	public static void paintAimedNode(Graphics2D a_gr, Node aimedNode)
    {
        paintAimedNode(a_gr, aimedNode, Color.RED);	
    }
    
    /**
     * plot searched possible ship positions
     * @param a_gr
     * @param possiblePosition
     */
    public static void paintPossibleShipPositions(Graphics2D a_gr, ArrayList<Vector2d> possiblePosition)
    {
    	a_gr.setColor(Color.lightGray);
//    	ArrayList<Vector2d> possiblePositionCopy = (ArrayList<Vector2d>) possiblePosition.clone();
    	if (null != possiblePosition)
    	{
    		for(Vector2d position : possiblePosition) 
        	{
    			a_gr.drawRect((int)position.x, (int)position.y, 1, 1);	
        	}	
    	}  	    	
    }
	public static void paintPossibleShipPositions(Graphics2D a_gr, HashMap<Vector2d, Double> possiblePositionScore) {		
    	if (null != possiblePositionScore)
    	{
    		for(Vector2d position : possiblePositionScore.keySet()) 
        	{
    			a_gr.setColor(Color.green);
    			if(possiblePositionScore.get(position) > 5)
    			{
    				a_gr.setColor(Color.orange);
    			}
    			if(possiblePositionScore.get(position) > 10)
    			{
    				a_gr.setColor(Color.red);
    			}
    			a_gr.drawRect((int)position.x, (int)position.y, 1, 1);	
        	}	
    	}
		
	}    
    
    /**
     * plot panic positions
     * @param a_gr
     * @param panicPosition
     */
	public static void paintPanicPositions(Graphics2D a_gr,
			ArrayList<Vector2d> panicPosition) {
    	a_gr.setColor(Color.red);
//    	ArrayList<Vector2d> possiblePositionCopy = (ArrayList<Vector2d>) panicPosition.clone();
    	if (null != panicPosition)
    	{
    		for(Vector2d position : panicPosition) 
        	{
    			a_gr.drawRect((int)position.x, (int)position.y, 5, 5);	
        	}	
    	} 
		
	}
    
    /**
     * draws a list of paths on the screen with the same color
     * @param a_gr
     * @param m_graph
     * @param m_pathList
     * @param color
     */
    public static void paintPaths(Graphics2D a_gr, Graph m_graph, ArrayList<Path> m_pathList, Color color)
    {
    	//paint planned paths
        if ( m_pathList.size() > 0) 
        {
	        for(int i = 0; i< m_pathList.size(); i++)
	        {	        	
	        	Path a_path = m_pathList.get(i);
	        	paintPath(a_gr, m_graph, a_path, color);
	        }
        }
    }
    
    /**
     * draws a single path on the screen
     * @param a_gr
     * @param m_graph
     * @param m_path
     * @param color
     */
    public static void paintPath(Graphics2D a_gr, Graph m_graph, Path m_path, Color color)
    {
    	a_gr.setColor(color);
    	if (null != m_path)
    	{
        	for(int j = 0; j < m_path.m_points.size()-1; ++j)
            {
                Node thisNode = m_graph.getNode(m_path.m_points.get(j));
                Node nextNode = m_graph.getNode(m_path.m_points.get(j+1));
                a_gr.drawLine(thisNode.x(), thisNode.y(), nextNode.x(),nextNode.y());
            }
    	}
    }
    
    
    
    //***********************************************************//
	public void waitRandom(int a_near)
    {
        Random m_rnd = new Random();
        int waitTime = (a_near-30) + m_rnd.nextInt(50);
        long startingTime = System.currentTimeMillis();
        long finalDateMs = startingTime + waitTime;

        while(startingTime < finalDateMs)
            startingTime = System.currentTimeMillis();

        System.out.println("Waited " + waitTime + " ms");
    }
	
	public void wait(int waitTime)
    {
        long startingTime = System.currentTimeMillis();
        long finalDateMs = startingTime + waitTime;

        while(startingTime < finalDateMs)
            startingTime = System.currentTimeMillis();

        System.out.println("Waited " + waitTime + " ms");
    }
}
