package controllers.keycontroller;
import java.awt.Color;
import java.awt.Graphics2D;

import framework.core.Controller;
import framework.core.Game;
import framework.utils.Vector2d;

/**
 * This class is used for the KeyController (human playing).
 * PTSP-Competition
 * Created by Diego Perez, University of Essex.
 * Date: 20/12/11
 */
public class KeyController extends Controller
{
    /**
     * To manage the keyboard input.
     */
    private KeyInput m_input;

    /**
     * Constructor of the KeyController.
     */
    public KeyController()
    {
        m_input = new KeyInput();
    }

    /**
     * This function is called every execution step to get the action to execute.
     * @param a_gameCopy Copy of the current game state.
     * @param a_timeDue The time the next move is due
     * @return the integer identifier of the action to execute (see interface framework.core.Controller for definitions)
     */
    public int getAction(Game a_gameCopy, long a_timeDue)
    {
    	Vector2d straightdown = new Vector2d();
    	straightdown.x = 0;
    	straightdown.y = 1;
    	straightdown.normalise();
    	Vector2d shipHeading;
    	shipHeading = a_gameCopy.getShip().d;
    	shipHeading_x = a_gameCopy.getShip().d.x;
    	shipHeading_y = a_gameCopy.getShip().d.y;
    	shipHeading.normalise();
    	System.out.println(shipHeading);
        System.out.println("heading (deg):" + Math.toDegrees(Math.acos(straightdown.dot(shipHeading))));
        return m_input.getAction();
    }
    
    /**
     * paint additional info
     */
    public void paint(Graphics2D a_gr)
    {
    	a_gr.setColor(Color.GRAY);
    	a_gr.drawString("x:" + Double.toString(shipHeading_x), 30,30);
    	a_gr.drawString("y:" + Double.toString(shipHeading_y), 30,60);
    }  

    /**
     * Return the input manager
     * @return the input manager
     */
    public KeyInput getInput() {return m_input;}
    
   private double shipHeading_x, shipHeading_y; 
}
