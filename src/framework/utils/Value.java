package framework.utils;

public class Value {
	public double distance;
	public double direction;
	public double value;
	
	public Value()
	{
		reset();
	}
	
	public void reset() {
		this.distance = Double.MAX_VALUE;
		this.direction= Double.MIN_VALUE;
		this.value =Double.MIN_VALUE;
	}
}