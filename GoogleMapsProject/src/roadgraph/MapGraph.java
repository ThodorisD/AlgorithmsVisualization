
package roadgraph;

import java.io.PrintWriter;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import geography.RoadSegment;
import util.GraphLoader;

public class MapGraph {

	// Maintain both nodes and edges as you will need to
	// be able to look up nodes by lat/lon or by roads
	// that contain those nodes.
	private HashMap<GeographicPoint,MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;


	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint,MapNode>();
		edges = new HashSet<MapEdge>();
	}

	public int getNumVertices()
	{
		return pointNodeMap.values().size();
	}

	public int getNumEdges()
	{
		return edges.size();
	}

	// For us in DEBUGGING.  Print the Nodes in the graph
	public void printNodes()
	{
		System.out.println("****PRINTING NODES ********");
		System.out.println("There are " + getNumVertices() + " Nodes: \n");
		for (GeographicPoint pt : pointNodeMap.keySet())
		{
			MapNode n = pointNodeMap.get(pt);
			System.out.println(n);
		}
	}

	// For us in DEBUGGING.  Print the Edges in the graph
	public void printEdges()
	{
		System.out.println("******PRINTING EDGES******");
		System.out.println("There are " + getNumEdges() + " Edges:\n");
		for (MapEdge e : edges)
		{
			System.out.println(e);
		}

	}


	public void addVertex(double latitude, double longitude)
	{
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}

	public void addVertex(GeographicPoint location)
	{
		MapNode n = pointNodeMap.get(location);
		if (n == null) {
			n = new MapNode(location);
			pointNodeMap.put(location, n);
		}
		else {
			System.out.println("Warning: Node at location " + location +
					" already exists in the graph.");
		}

	}

	public void addEdge(double lat1, double lon1,
						double lat2, double lon2, String roadName, String roadType)
	{
		// Find the two Nodes associated with this edge.
		GeographicPoint pt1 = new GeographicPoint(lat1, lon1);
		GeographicPoint pt2 = new GeographicPoint(lat2, lon2);

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);

	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType) {

		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType, double length) {
		MapNode n1 = pointNodeMap.get(pt1);
		MapNode n2 = pointNodeMap.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, length);
	}

	/** Given a point, return if there is a corresponding MapNode **/
	public boolean isNode(GeographicPoint point)
	{
		return pointNodeMap.containsKey(point);
	}



	// Add an edge when you already know the nodes involved in the edge
	private void addEdge(MapNode n1, MapNode n2, String roadName,
			String roadType,  double length)
	{
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}


	/** Returns the nodes in terms of their geographic locations */
	public Collection<GeographicPoint> getVertices() {
		return pointNodeMap.keySet();
	}

	// get a set of neighbor nodes from a mapnode
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}

	private List<GeographicPoint>
	reconstructPath(HashMap<MapNode,MapNode> parentMap,
					MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}


	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {

        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	

	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin dijkstra
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		for(GeographicPoint gp : pointNodeMap.keySet()){
			pointNodeMap.get(gp).setDistance(Double.POSITIVE_INFINITY);
		}
		
		startNode.setDistance(0);
		toExplore.add(startNode);
		MapNode next = null;

		int removedItems = 0;
		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			removedItems++;
			 // hook for visualization
			nodeSearched.accept(next.getLocation());
			if(!visited.contains(next)){
				
				visited.add(next);
				
				if (next.equals(endNode)) break;
				Set<MapNode> neighbors = getNeighbors(next);
				for (MapNode neighbor : neighbors) {
					if(!visited.contains(neighbor)){
						for(MapEdge mapEdge : next.getEdges()){
							if(mapEdge.getEndNode().equals(neighbor)){
								neighbor.setActualDistance(next.getDistance() + mapEdge.getLength());
								break;
							}
						}
						
						if(neighbor.getActualDistance() < neighbor.getDistance()){
							neighbor.setDistance(neighbor.getActualDistance());
							if(parentMap.get(neighbor) != null){
								parentMap.remove(neighbor);
							}
							parentMap.put(neighbor, next);
						}
						toExplore.add(neighbor);
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		System.out.println("Remove Items " + removedItems);
		
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);

		return path;

		
	}

	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}

	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}

		// setup to begin Astar
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		for(GeographicPoint gp : pointNodeMap.keySet()){
			pointNodeMap.get(gp).setDistance(Double.POSITIVE_INFINITY);
			pointNodeMap.get(gp).setActualDistance(Double.POSITIVE_INFINITY);
		}
		
		startNode.setDistance(0);
		startNode.setActualDistance(0);
		toExplore.add(startNode);
		MapNode next = null;
		
		int removedItems = 0;
		while (!toExplore.isEmpty()) {
			next = toExplore.remove();
			removedItems++;
			 // hook for visualization
			nodeSearched.accept(next.getLocation());
			if(!visited.contains(next)){
				
				visited.add(next);
				
				if (next.equals(endNode)) break;
				Set<MapNode> neighbors = getNeighbors(next);
				for (MapNode neighbor : neighbors) {
					if(!visited.contains(neighbor)){
						for(MapEdge mapEdge : next.getEdges()){
							if(mapEdge.getEndNode().equals(neighbor)){
								neighbor.setActualDistance(next.getDistance() + mapEdge.getLength());
								break;
							}
						}
						
						double distanceStraight = neighbor.getLocation().distance(endNode.getLocation());
						if(neighbor.getActualDistance() + distanceStraight < neighbor.getDistance()){
							neighbor.setDistance(neighbor.getActualDistance() + distanceStraight);
							if(parentMap.get(neighbor) != null){
								parentMap.remove(neighbor);
							}
							parentMap.put(neighbor, next);
						}
						toExplore.add(neighbor);
					}
				}
			}
		}
		if (!next.equals(endNode)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		
		System.out.println("Remove Items " + removedItems);
		
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentMap, startNode, endNode);

		return path;
		
		
	}
	
	private double straightDistance(GeographicPoint start, GeographicPoint end){
		return Math.sqrt(Math.pow(Math.abs(start.getX()-end.getX()),2) * Math.pow(Math.abs(start.getY()-end.getY()),2));
	}

}

