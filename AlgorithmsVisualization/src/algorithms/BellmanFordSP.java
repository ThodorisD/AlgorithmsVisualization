package algorithms;

import org.graphstream.graph.Graph;
import org.graphstream.graph.Node;

public class BellmanFordSP {
    private double[] distTo;               // distTo[v] = distance  of shortest s->v path
    private DirectedEdge[] edgeTo;         // edgeTo[v] = last edge on shortest s->v path
    private boolean[] onQueue;             // onQueue[v] = is v currently on the queue?
    private Queue<Integer> queue;          // queue of vertices to relax
    private int cost;                      // number of calls to relax()
    private Iterable<DirectedEdge> cycle;  // negative cycle (or null if no such cycle)
    
    private Graph graph ;

    public BellmanFordSP(Graph graph,EdgeWeightedDigraph G,int s) {
    	
    	this.graph = graph ;
        distTo  = new double[G.V()];
        edgeTo  = new DirectedEdge[G.V()];
        onQueue = new boolean[G.V()];
        
        
        for (int v = 0; v < G.V(); v++) {
            distTo[v] = Double.POSITIVE_INFINITY;
            
        	Node graphNode = graph.getNode(Integer.toString(v)) ;
        	graphNode.addAttribute("ui.label", "           " +Integer.toString(v) + "       [" + Double.POSITIVE_INFINITY + "]");
        }
        
        distTo[s] = 0.0;
        
        Node graphNode = graph.getNode(Integer.toString(s)) ;
        graphNode.addAttribute("ui.label", "        " +Integer.toString(s) + "       ["+distTo[s]+"]");
        
        graph.getNode(Integer.toString(s)).addAttribute("ui.class", "start");
        sleep(1500);
        
        // Bellman-Ford algorithm
        queue = new Queue<Integer>();
        queue.enqueue(s);
        onQueue[s] = true;
        while (!queue.isEmpty() && !hasNegativeCycle()) {
            int v = queue.dequeue();
            onQueue[v] = false;
            relax(G, v);
        }
    }

    
    private void relax(EdgeWeightedDigraph G, int v) {
        for (DirectedEdge e : G.adj(v)) {
            int w = e.to();
            
            graph.getEdge(e.from() + "" + w).addAttribute("ui.class", "current");
            //graph.getNode(Integer.toString(w)).addAttribute("ui.class", "current");

            sleep(1500);
            
            if (distTo[w] > distTo[v] + e.weight()) {
                distTo[w] = distTo[v] + e.weight();
                
                Node nodeRelax = graph.getNode(Integer.toString(w)) ;
                nodeRelax.addAttribute("ui.label", "        " +Integer.toString(w) + "       ["+distTo[w]+"]");

                
                edgeTo[w] = e;
                if (!onQueue[w]) {
                    queue.enqueue(w);
                    onQueue[w] = true;
                }
            }
            if (cost++ % G.V() == 0) {
                findNegativeCycle();
                if (hasNegativeCycle()) return;  // found a negative cycle
            }
            
            graph.getEdge(e.from() + "" + w).addAttribute("ui.class", "visited");
        }
    }


    public boolean hasNegativeCycle() {
        return cycle != null;
    }

    public Iterable<DirectedEdge> negativeCycle() {
        return cycle;
    }

    // by finding a cycle in predecessor graph
    private void findNegativeCycle() {
        int V = edgeTo.length;
        EdgeWeightedDigraph spt = new EdgeWeightedDigraph(V);
        for (int v = 0; v < V; v++)
            if (edgeTo[v] != null)
                spt.addEdge(edgeTo[v]);

        EdgeWeightedDirectedCycle finder = new EdgeWeightedDirectedCycle(spt);
        cycle = finder.cycle();
        
        if(cycle != null)
        {
	        for(DirectedEdge e: cycle) {
	        	String edge = e.from() + "" + e.to() ;
	        	graph.getEdge(edge).addAttribute("ui.class", "start");
	        	
	            graph.getNode(Integer.toString(e.from())).addAttribute("ui.class", "start");
	            graph.getNode(Integer.toString(e.to())).addAttribute("ui.class", "start");
	            sleep(1500);
	        }
        }
        
    }

    public double distTo(int v) {
        if (hasNegativeCycle())
            throw new UnsupportedOperationException("Negative cost cycle exists");
        return distTo[v];
    }


    public boolean hasPathTo(int v) {
        return distTo[v] < Double.POSITIVE_INFINITY;
    }

    public Iterable<DirectedEdge> pathTo(int v) {
        if (hasNegativeCycle())
            throw new UnsupportedOperationException("Negative cost cycle exists");
        if (!hasPathTo(v)) return null;
        Stack<DirectedEdge> path = new Stack<DirectedEdge>();
        for (DirectedEdge e = edgeTo[v]; e != null; e = edgeTo[e.from()]) {
        	
        	String edge = e.from() + "" + e.to() ;
        	graph.getEdge(edge).addAttribute("ui.class", "start");
        	
            graph.getNode(Integer.toString(e.from())).addAttribute("ui.class", "start");
            graph.getNode(Integer.toString(e.to())).addAttribute("ui.class", "start");
            sleep(1500);
            
            path.push(e);

        }
        return path;
    }
    
    public void sleep(long ms) {
  		try { Thread.sleep(ms); } catch (Exception e) {e.printStackTrace();}
  	}
}