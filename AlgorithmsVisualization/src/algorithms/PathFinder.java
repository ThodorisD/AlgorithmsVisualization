package algorithms;

import org.graphstream.graph.Node;

import java.awt.Component;
import java.awt.Dimension;

import javax.swing.JFrame;

import org.graphstream.graph.Edge;
import org.graphstream.graph.Graph;
import org.graphstream.graph.implementations.SingleGraph;
import org.graphstream.ui.view.View;
import org.graphstream.ui.view.Viewer;

import parser.DataParser;


public class PathFinder {
	private static EdgeWeightedDigraph edgeWeightedDigraph ;
	
	private static Graph graph;
	private static DataParser parser;		// parser used to parse the file with the graph data.
	private long DELAY = 1500;				// sets visualization delay
	private Viewer viewer ;
	
    public EdgeWeightedDigraph initializeGraph() {
		
    	// Create Frame
    	JFrame myFrame = new JFrame("Visualization");
		myFrame.setLocation(100, 100);
		myFrame.setSize(new Dimension(1024, 800));

		createGrpah();

		graph = new SingleGraph("AlgorithmsVisalization");
		graph.setAutoCreate(true);
		graph.setStrict(false);
		
		Viewer viewer = new Viewer(graph, Viewer.ThreadingModel.GRAPH_IN_ANOTHER_THREAD);
		viewer.setCloseFramePolicy(Viewer.CloseFramePolicy.HIDE_ONLY);
		View view = viewer.addDefaultView(false);
		
		graph.addAttribute("ui.quality");
		graph.addAttribute("ui.antialias");
		graph.addAttribute("ui.stylesheet", "url('resources/style.css')");

		
		setupGraphNodes();
		setupGraphEdges() ;

		viewer.enableAutoLayout();

		myFrame.add((Component) view);
		myFrame.setVisible(true);
		myFrame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);
		
		return edgeWeightedDigraph;
    }
    

	private static void createGrpah() {
		parser = new DataParser();
		edgeWeightedDigraph = parser.getGraph();
	}
	

	private void setupGraphNodes() {

		for(int i=0 ; i < edgeWeightedDigraph.V() ; i++) {
			// Add node to graph
			Node graphNode = graph.addNode(Integer.toString(i));
			graphNode.addAttribute("ui.label",Integer.toString(i));
		}		
	}
	
	private void setupGraphEdges() {
		
		for(int i = 0 ; i < edgeWeightedDigraph.V() ;i++) {
			for(DirectedEdge edge : edgeWeightedDigraph.adj(i)) {
				
				// Add edge to graph
				String nodeName = Integer.toString(edge.from()) ;
				String nodeName2 = Integer.toString(edge.to()) ;
				String edgeName = nodeName + nodeName2 ;

				Edge e = graph.addEdge(edgeName, nodeName, nodeName2,true);
				e.addAttribute("ui.label",String.valueOf(edge.weight()));
			}
		}
	}
	
	public BellmanFordSP startBellmanFord(int startNode) {
    	clearNodeColors();
		BellmanFordSP bellmanFordSP = new BellmanFordSP(graph,edgeWeightedDigraph, startNode) ;
		
		return bellmanFordSP ;
	}

	public Viewer getViewer() {
		return this.viewer ;
	}
	
	
	public void clearNodeColors() {
		for (Node node : graph.getNodeSet()) {
			node.removeAttribute("ui.class");
		}
		
		for (Edge edge :graph.getEdgeSet()) {
			edge.removeAttribute("ui.class");
		}
	}
	
	public void sleep(long ms) {
		try { Thread.sleep(ms); } catch (Exception e) {e.printStackTrace();}
	}

	public void setVisualizationDelay(int value) {
		this.DELAY = value;
	}

	// Change graph font size
	public void setGraphFontSize(int fontSize) {
		for (org.graphstream.graph.Node node : graph.getNodeSet()) {
			node.addAttribute("ui.style", "size: " + fontSize + "px;");
			node.addAttribute("ui.style", "text-size: " + (fontSize - 5) + "px;");
		}

		for (Edge edge : graph.getEdgeSet()) {
			edge.addAttribute("ui.style", "text-size: " + (fontSize - 5) + "px;");
		}
	}
}