import java.io.*;
import java.util.*;

class bfs{
	int algo, no_nodes;
	String src, dest;
	String[] nodes = new String[no_nodes];
	int matrix[][] = new int[no_nodes][no_nodes];
	String outpath;
	FileWriter writer = null;
	List path_list = new LinkedList();
	public bfs(String src2, String dest2, int algo2, int no_nodes2,
			int[][] matrix2, String[] nodes2) {

		algo = algo2;
		no_nodes = no_nodes2;
		src = src2;
		dest = dest2;
		matrix = matrix2;
		nodes = nodes2;
		

	}

	public void printPath(int[] parent, PrintWriter pw) throws IOException
    {
		int destination = Arrays.asList(nodes).indexOf(dest);
		int source = Arrays.asList(nodes).indexOf(src);
        path_list.add(dest);
        boolean found = false;
        int vertex = destination;
        while (!found)
        {
            if (vertex == source)
            {
                found = true;
                continue;
            }
            path_list.add(nodes[parent[vertex]]);
            vertex = parent[vertex];
        }
        for(int i=path_list.size()-1;i>0;i--)
        	pw.write(path_list.get(i)+"-");
        	pw.write(path_list.get(0)+"");
     
    }
public void bfs() throws IOException {
	
	int explored[] = new int[no_nodes];
    final int[] path_cost = new int[no_nodes];
    int [] parent = new int[no_nodes];
	int [] parent_real = new int[no_nodes];
	int depth = 0;
	
	//File f = new File("Output.txt");
	
	File outFile = new File ("output.txt");
    FileWriter fWriter = new FileWriter (outFile);
    PrintWriter pw = new PrintWriter (fWriter);   
	PriorityQueue<String> q = new PriorityQueue<String>(20,
				            new Comparator<String>() {
				                // overriding the compare method
								
				                public int compare(String i, String j) {
				                	int a = Arrays.asList(nodes).indexOf(i);
				                	int b = Arrays.asList(nodes).indexOf(j);
				                	if(path_cost[a]==path_cost[b])
				                    return i.compareTo(j);
				                	if(path_cost[a]>path_cost[b])
				                		return 1;
									return 0;
				                }
		 }     );
			
	String log = "", path = "";
	q.add(src);
	Arrays.fill(explored, 0); // set the explored array to be 0 (unvisited)
//	Arrays.fill(parent, 0);
	String node = src;
	int src_index = 0;
	int index, dest_index;
	String obj = null;
	dest_index = Arrays.asList(nodes).indexOf(dest);
	while (!q.isEmpty())// visited
	{
		
		obj = q.poll(); // removing from queue
		
		index = Arrays.asList(nodes).indexOf(obj);
		if (explored[index] == 0) {
			explored[index] = 1;
			
		}
		log = log + nodes[index];
		if (obj.equals(dest))
			break;
		else
			log = log + "-";
	
		src_index = Arrays.asList(nodes).indexOf(src);
		
		path_cost[src_index]=0;
		
		List children = new LinkedList();

	

		for (int j = 0; j < no_nodes; j++) {
			if (matrix[index][j] != 0 && explored[j] != 1
					&& (!q.contains(nodes[j]))) // queue contents
				{
				children.add(nodes[j]);
			
				parent[j]=Arrays.asList(nodes).indexOf(obj);
				int cur_node_index = Arrays.asList(nodes).indexOf(obj);
				path_cost[j]=path_cost[cur_node_index]+matrix[cur_node_index][j];
				}
		}
	

		for (int m = 0; m < children.size(); m++) {
			q.add((String) children.get(m));
		}			
	}
	
	
	if(!log.contains(dest))
		pw.write("NoPathAvailable");
		
	else
	{
	pw.print(log);
	pw.print("\n");		
	printPath(parent,pw);
	pw.print("\n");
	int destination = Arrays.asList(nodes).indexOf(dest);
	pw.print(path_cost[destination]);
	pw.close();
	System.out.println();
	
	}	

}
}
class dfs {
	int algo, no_nodes;
	String src, dest;
	String[] nodes = new String[no_nodes];
	int matrix[][] = new int[no_nodes][no_nodes];
	List path_list = new LinkedList();
	public dfs(String src2, String dest2, int algo2, int no_nodes2,
			int[][] matrix2, String[] nodes2) {

		algo = algo2;
		no_nodes = no_nodes2;
		src = src2;
		dest = dest2;
		matrix = matrix2;
		nodes = nodes2;

	}
	public void printPath(int[] parent, PrintWriter pw)
    {
		int destination = Arrays.asList(nodes).indexOf(dest);
		int source = Arrays.asList(nodes).indexOf(src);
        path_list.add(dest);
        boolean found = false;
        int vertex = destination;
        while (!found)
        {
            if (vertex == source)
            {
                found = true;
                continue;
            }
            path_list.add(nodes[parent[vertex]]);
            vertex = parent[vertex];
        }
        for(int i=path_list.size()-1;i>0;i--)
        	pw.print(path_list.get(i)+"-");
        pw.print(path_list.get(0));
     
    }

	
	public void dfs() throws IOException {
		File outFile = new File ("output.txt");
	    FileWriter fWriter = new FileWriter (outFile);
	    PrintWriter pw = new PrintWriter (fWriter); 
		int explored[] = new int[no_nodes];
		int[] path_cost = new int[no_nodes];
		int [] parent = new int[no_nodes];
		int [] parent_real = new int[no_nodes];
		int depth = 0;
		Stack stack = new Stack();
		String log = "", path = "";
		stack.push(src);
		Arrays.fill(explored, 0); // set the explored array to be 0 (unvisited)
		String node = src;
		int index, dest_index;
		String obj = null;
		dest_index = Arrays.asList(nodes).indexOf(dest);
		String [] stack_contents = new String [stack.size()];
		ArrayList<String> sort_stack = new ArrayList<String>();
		int src_index;
		while (!stack.isEmpty())// visited
		{
			
			obj = (String) stack.pop();
			index = Arrays.asList(nodes).indexOf(obj); // node to be explored
			// System.out.print(obj);

			// if(explored[index]==0)

			explored[index] = 1;

			// log = log + nodes[index];
			log = log + obj;
			if (obj.equals(dest))
				break;
			else
				log = log + "-";
			List children = new LinkedList();
			src_index = Arrays.asList(nodes).indexOf(src);
			
			path_cost[src_index]=0;
			

			// System.out.println(path);

			for (int j = 0; j < no_nodes; j++) {
				if (matrix[index][j] != 0 && explored[j] != 1
						&& (!stack.contains(nodes[j]))) // queue contents
					{
					children.add(nodes[j]);
					parent[j]=Arrays.asList(nodes).indexOf(obj);
					int cur_node_index = Arrays.asList(nodes).indexOf(obj);
					path_cost[j]=path_cost[cur_node_index]+matrix[cur_node_index][j];
					}				
			}
			
			
			//java.util.Collections.sort(children);

			for (int m = children.size() - 1; m >= 0; m--) {
				stack.push((String) children.get(m));
			}
			
			
			//System.out.println(stack.pop());
		}

		
		if(!log.contains(dest))
			pw.println("NoPathAvailable");
		else
		{
		pw.print(log);
		pw.print("\n");
		printPath(parent,pw);
		pw.print("\n");
		int destination = Arrays.asList(nodes).indexOf(dest);
		pw.print(path_cost[destination]);
		pw.close();
		}

	}

}
 class ucs {
	int algo, no_nodes;
	String src, dest;
	String[] nodes = new String[no_nodes];
	int matrix[][] = new int[no_nodes][no_nodes];
	
	List path_list = new LinkedList();
	public ucs(String src2, String dest2, int algo2, int no_nodes2,
			int[][] matrix2, String[] nodes2) {

		algo = algo2;
		no_nodes = no_nodes2;
		src = src2;
		dest = dest2;
		matrix = matrix2;
		nodes = nodes2;

	}
	public void printPath(int[] parent, PrintWriter pw)
    {
		int destination = Arrays.asList(nodes).indexOf(dest);
		int source = Arrays.asList(nodes).indexOf(src);
        path_list.add(dest);
        boolean found = false;
        int vertex = destination;
        while (!found)
        {
            if (vertex == source)
            {
                found = true;
                continue;
            }
            path_list.add(nodes[parent[vertex]]);
            vertex = parent[vertex];
        }
        for(int i=path_list.size()-1;i>0;i--)
        	pw.print(path_list.get(i)+"-");
       pw.print(path_list.get(0));
     
    }
	public void ucs() throws IOException {
		int explored[] = new int[no_nodes];
	    final int[] path_cost = new int[no_nodes];
	    int [] parent = new int[no_nodes];
		int [] parent_real = new int[no_nodes];
		int depth = 0;
		File outFile = new File ("output.txt");
	    FileWriter fWriter = new FileWriter (outFile);
	    PrintWriter pw = new PrintWriter (fWriter); 
		PriorityQueue<String> q = new PriorityQueue<String>(20,
					            new Comparator<String>() {
					                // overriding the compare method
									
					                public int compare(String i, String j) {
					                	int a = Arrays.asList(nodes).indexOf(i);
					                	int b = Arrays.asList(nodes).indexOf(j);
					                	if(path_cost[a]==path_cost[b])
					                    return i.compareTo(j);
					                	if(path_cost[a]>path_cost[b])
					                		return 1;
					                	if(path_cost[a]<path_cost[b])
					                		return -1;
										return 0;
					                }
			 }     );
				
		String log = "", path = "";
		q.add(src);
		Arrays.fill(explored, 0); // set the explored array to be 0 (unvisited)
	//	Arrays.fill(parent, 0);
		String node = src;
		int src_index = 0;
		int index, dest_index;
		String obj = null;
		dest_index = Arrays.asList(nodes).indexOf(dest);
		while (!q.isEmpty())// visited
		{
			
			obj = q.poll(); // removing from queue
			
			index = Arrays.asList(nodes).indexOf(obj);
			if (explored[index] == 0) {
				explored[index] = 1;
				
			}
			log = log + nodes[index];
			if (obj.equals(dest))
				break;
			else
				log = log + "-";
		
			src_index = Arrays.asList(nodes).indexOf(src);
			
			path_cost[src_index]=0;
			
			List children = new LinkedList();

		

			for (int j = 0; j < no_nodes; j++) {
				if (matrix[index][j] != 0 && explored[j] != 1
						&& (!q.contains(nodes[j]))) // queue contents
					{
					children.add(nodes[j]);
				
					parent[j]=Arrays.asList(nodes).indexOf(obj);
					int cur_node_index = Arrays.asList(nodes).indexOf(obj);
					path_cost[j]=path_cost[cur_node_index]+matrix[cur_node_index][j];
					}
			}
			

			for (int m = 0; m < children.size(); m++) {
				q.add((String) children.get(m));
			}			
		}
		
		
		if(!log.contains(dest))
			pw.println("NoPathAvailable");
		else
		{
	    pw.println(log);
		printPath(parent,pw);
		pw.println("");
		int destination = Arrays.asList(nodes).indexOf(dest);
		pw.println(path_cost[destination]);
		pw.close();
		}
	}

}

public class ucSearch {

	
	public static void main(String args[]) throws IOException {
		FileReader fileReader = new FileReader("input.txt");
		BufferedReader bufferedReader = new BufferedReader(fileReader);
		List<String> lines = new ArrayList<String>();
		String line = null;
		int algo, no_nodes;
		String src = null, dest = null;
		String file_write = null;
		while ((line = bufferedReader.readLine()) != null) {
			lines.add(line);
		}
		bufferedReader.close();
		lines.toArray(new String[lines.size()]);
		algo = Integer.parseInt(lines.get(0));
		src = lines.get(1);
		dest = lines.get(2);
		no_nodes = Integer.parseInt(lines.get(3));
		int limit = no_nodes + 4, counter = 0;
		String nodes[] = new String[no_nodes];
		for (int i = 4; i < limit; i++) {
			if (counter == nodes.length)
				break;
			nodes[counter] = lines.get(i);
			counter++;
		}
		int rows = no_nodes;
		int cols = no_nodes;
		String arr = "";
		int[] intarr = new int[rows * cols];
		// List<List<Integer>> main_matrix = new
		// ArrayList<List<Integer>>(outer_size);
		for (int i = limit; i < lines.size(); i++) {

			arr = arr + (lines.get(i)) + " ";

		}
		String[] array = (arr.split(" "));
		for (int i = 0; i < array.length; i++) {

			intarr[i] = Integer.parseInt(array[i]);

		}

		int matrix[][] = new int[rows][cols];
		int count = 0;
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				if (count == intarr.length)
					break;
				matrix[i][j] = intarr[count];
				count++;
			}
		}
		
		if(algo==1)
		{
		bfs b = new bfs(src, dest, algo, no_nodes, matrix, nodes);
		b.bfs();
		}
		System.out.println();
		if(algo==2)
		{
		dfs d = new dfs(src, dest, algo, no_nodes, matrix, nodes);
		d.dfs();
		}
		System.out.println();
		if(algo==3)
		{
		ucs u = new ucs(src, dest, algo, no_nodes, matrix, nodes);
		u.ucs();
		}

	}
}
