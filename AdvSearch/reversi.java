import java.io.*;
import java.util.*;
import java.util.concurrent.CopyOnWriteArrayList;

public class reversi
{
	 final static int[][] pos_matrix = 
			{   {99, -8, 8, 6, 6, 8, -8, 99},
	            {-8, -24, -4, -3, -3, -4, -24, -8},
	            {8, -4, 7, 4, 4, 7, -4, 8},
	            {6, -3, 4, 0, 0, 4, -3, 6},
	            {6, -3, 4, 0, 0, 4, -3, 6},
	            {8, -4, 7, 4, 4, 7, -4, 8},
	            {-8, -24, -4, -3, -3, -4, -24, -8},
	            {99, -8, 8, 6, 6, 8, -8, 99}};      //positional_weight matrix 
	
	  static int cut_off,task;static String maxPlayer;
	  static List<String> print_log_al = new ArrayList<String>();
	  static List<String> print_log_mm = new ArrayList<String>();
	static String minPlayer;
          
	public static void main(String args[]) throws IOException {
		FileReader fileReader = new FileReader("/Users/anupamamukund/Documents/USC/CS561/Homework/Homework2/check.txt");
		BufferedReader bufferedReader = new BufferedReader(fileReader);
		List<String> lines = new ArrayList<String>();	//lines - to store file input
		String line = null;	//to read each line
		
		int no_nodes = 64;
		char myPlayer, opponent;
		String myPlayer_string;
		while ((line = bufferedReader.readLine()) != null) {
			lines.add(line);
		}
		bufferedReader.close();
		lines.toArray(new String[lines.size()]);
		task = Integer.parseInt(lines.get(0));
		myPlayer_string = lines.get(1);
		String board[][] = new String[8][8]
		if(myPlayer_string.equalsIgnoreCase("X"))   //store maxPlayer and minPlayer
		{
			opponent = 'O';
			myPlayer = 'X';
		}
		else
			{
				opponent = 'X';
				myPlayer = 'O';
			}
		maxPlayer = Character.toString(myPlayer);
		minPlayer = Character.toString(opponent);
		cut_off = Integer.parseInt(lines.get(2));   //obtain cut off
		
		String arr="";
		for(int i=3;i<lines.size();i++)
		{
			arr = arr + lines.get(i);
		
		}
		int ch=0;
		for(int i=0;i<8;i++)
		{
			for(int j=0;j<8;j++)
			{
				char temp = arr.charAt(ch);
				board[i][j]=Character.toString(temp);   //store the input matrix
		
				ch++;
			}
		}
		
		if(task == 1)
			greedy(board, maxPlayer, minPlayer);
		if(task == 2)
			minimax(board, maxPlayer, minPlayer);
		if(task == 3)
			alpha_beta(board, maxPlayer, minPlayer);
		
	}

	private static void alpha_beta(String[][] board, String maxPlayer2,
			String minPlayer2) throws IOException {
		File outFile = new File ("output.txt");
	    FileWriter fWriter = new FileWriter (outFile);
	    PrintWriter pw = new PrintWriter (fWriter);  
		ParentBoard parent_a = new ParentBoard();           //create parent board
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				parent_a.arr[i][j] = board[i][j];
		parent_a.depth = 0; parent_a.h = Integer.MIN_VALUE; //initialize depth and height of parent board
		
        //create children and grand-children for parent
        get_children(parent_a,maxPlayer,minPlayer,parent_a.depth,cut_off);
        
        //order all children position-wise
		Children_ordering(parent_a,parent_a.order_children);
		int alpha = Integer.MIN_VALUE; int beta = Integer.MAX_VALUE;
		//invoke alpha-beta algorithm
        int alpha_final = alphabeta(parent_a,cut_off,alpha,beta,true);
		for(ParentBoard ch:parent_a.order_children)
		{
			if(ch.h == alpha_final)
			{
				for(int i=0;i<8;i++)
				{
					for(int j=0;j<8;j++)
					{
						pw.write(ch.arr[i][j]);
					}
					pw.write("\n");
				}
			}
			break;
		}
		pw.write("Node,Depth,Value,Alpha,Beta\n");
		for(int i=0;i<print_log_al.size();i++)
			pw.write(print_log_al.get(i));
		pw.flush();
		
	}

	private static int alphabeta(ParentBoard p, int depth, int alpha,
			int beta, boolean maximizingPlayer) {
		
		if(p.depth == 0 && p.h == Integer.MIN_VALUE)
			print_log_al.add("root"+","+p.depth+","+"-Infinity"+"-Infinity"+",Infinity\n");
		
		
		else 
		{
			if(p.depth!=0 && p.h == Integer.MIN_VALUE && alpha == Integer.MIN_VALUE && beta == Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",Infinity,-Infinity"+",Infinity\n");
			if(p.h == Integer.MAX_VALUE && alpha ==Integer.MIN_VALUE && beta == Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+"Infinity,-Infinity,Infinity\n");
			else if(p.h == Integer.MAX_VALUE && beta == Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",Infinity,"+alpha+","+"Infinity\n");
			else
			{
				if(p.h!=Integer.MIN_VALUE && alpha == Integer.MIN_VALUE && beta == Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+",-Infinity"+",Infinity\n");
				
				else if(p.h!=0 && alpha==Integer.MIN_VALUE && beta != Integer.MAX_VALUE)
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+","+"-Infinity,"+beta+"\n");
				else if(alpha == Integer.MIN_VALUE)
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+","+"-Infinity,"+beta+"\n");
				else if(beta == Integer.MAX_VALUE && p.depth!=0 && p.h!=Integer.MIN_VALUE)
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+","+alpha+",Infinity\n");
				else if(beta == Integer.MAX_VALUE && p.depth!=0 && p.h==Integer.MIN_VALUE)
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",-Infinity"+","+alpha+",Infinity\n");
					else
						print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+","+alpha+","+beta+"\n");	
			}
		}
		if(p.depth == cut_off)
	return p.h;

if(maximizingPlayer)
{
	
	for(ParentBoard c:p.order_children)
	{
		alpha = Math.max(alpha, alphabeta(c,cut_off-1,alpha,beta,false));
		p.h = alpha;
		if(p.depth ==0 && alpha==Integer.MIN_VALUE && beta == Integer.MAX_VALUE && p.h == Integer.MIN_VALUE)
			print_log_al.add("root"+","+p.depth+",-Infinity, -Infinity, Infinity\n");
		if(p.depth ==0 && alpha==Integer.MIN_VALUE && beta!=Integer.MAX_VALUE)
			print_log_al.add("root,"+p.depth+","+p.h+","+"-Infinity,"+beta+"\n");
		if(p.depth == 0 && beta == Integer.MAX_VALUE)
			print_log_al.add("root,"+p.depth+","+p.h+","+alpha+","+"Infinity\n");
		else
		{
			if(p.h == Integer.MIN_VALUE && p.depth!=0 && alpha == Integer.MIN_VALUE && beta == Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",-Infinity"+",-Infinity,Infinity\n");
			else if(p.h == Integer.MAX_VALUE && alpha == Integer.MIN_VALUE && beta == Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+"Infinity"+"-Infinity,Infinity\n");
			else
				if(alpha==Integer.MIN_VALUE)
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+",-Infinity"+beta+"\n");
			else
					if(beta==Integer.MAX_VALUE)
						print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+","+alpha+",Infinity\n");
					
					else if((p.h==Integer.MAX_VALUE) && (beta == Integer.MAX_VALUE) && (p.depth!=0) )
						print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",Infinity"+","+alpha+"Infinity\n");
				else
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+","+alpha+","+beta+"\n");
		}
		if(beta <= alpha)
			break;
	}
	return alpha;
}
else
{
	
	for(ParentBoard c:p.order_children)
	{
		beta = Math.min(beta, alphabeta(c,cut_off-1,alpha,beta,true));
		p.h = beta;
		if(p.depth ==0)
			print_log_al.add("root"+","+p.depth+","+p.h+"\n");
		if(p.depth ==0 && alpha==Integer.MIN_VALUE && beta!=Integer.MAX_VALUE)
			print_log_al.add("root,"+p.depth+","+p.h+","+"-Infinity,"+beta+"\n");
		
		else
		{
			if(p.h == Integer.MAX_VALUE && p.depth!=0 && alpha == Integer.MIN_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",-Infinity"+",-Infinity,"+beta+"\n");
			else if(p.h == Integer.MAX_VALUE && alpha == Integer.MIN_VALUE && beta ==Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+"Infinity"+",-InfinitInfinity\n");
			else if(p.h == Integer.MAX_VALUE && beta == Integer.MAX_VALUE)
				print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+"Infinity,"+alpha+",Infinity\n");
			else
				if(alpha == Integer.MIN_VALUE && p.depth!=0)
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+",-Infinity"+","+beta+"\n");
				else
					print_log_al.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+","+alpha+","+beta+"\n");
		}
		if(beta <= alpha)
			break;
	}
	return beta;
}
	}

	private static void minimax(String[][] board, String maxPlayer2,
			String minPlayer2) throws IOException {
		File outFile = new File ("output.txt");
	    FileWriter fWriter = new FileWriter (outFile);
	    PrintWriter pw = new PrintWriter (fWriter);  	
	ParentBoard parent_m = new ParentBoard();
	for(int i=0;i<8;i++)
		for(int j=0;j<8;j++)
			parent_m.arr[i][j] = board[i][j];		//create root
	parent_m.depth = 0;
	parent_m.h = Integer.MIN_VALUE;
	get_children(parent_m,maxPlayer,minPlayer,parent_m.depth,cut_off);
	Children_ordering(parent_m, parent_m.order_children);
	int h = (minimax(parent_m,parent_m.depth,true));
	for(ParentBoard c:parent_m.order_children)
	{
		if(c.h == h)
		{
			for(int i=0;i<8;i++)
			{
				for(int j=0;j<8;j++)
				{
					pw.write(c.arr[i][j]);
				}
				pw.write("\n");
			}
		}
		break;
	}
	pw.write("Node,Depth,Value\n");
	for(int i=0;i<print_log_mm.size();i++)
	{
		pw.write(print_log_mm.get(i));
	}
		pw.flush();
	}
	
	public static String alphabetical_rc_ordering(int row, int col)
	{
		int i = row+1;
		int j = col+1;
		String acol ="";
		if(j==1)
			acol = "a";
		if(j==2)
			acol = "b";
		if(j==3)
			acol = "c";
		if(j==4)
			acol = "d";
		if(j==5)
			acol = "e";
		if(j==6)
			acol = "f";
		if(j==7)
			acol = "g";
		if(j==8)
			acol = "h";
		return acol+""+i;
		
	}
	@SuppressWarnings("unchecked")
	public static void Children_ordering(ParentBoard p, ArrayList<ParentBoard> order_children)
	{
		if(p.depth == cut_off)
			return;
		else
		{
			for(ParentBoard c: p.children)
			{
				order_children.add(c);
				Collections.sort(order_children, new Comparator<ParentBoard>(){
							public int compare(ParentBoard p1, ParentBoard p2){
								if(p1.depth!=p2.depth)
									return p1.depth - p2.depth;
								else
									if(p1.depth == p2.depth)
							{	
								if(p1.rowP == p2.rowP)
								{
									return (p1.colP - p2.colP);
								}
								else
									return (p1.rowP - p2.rowP);
				

								}
								return 0;
							}
			});
				Children_ordering(c,c.order_children);
			}
			
		}
		
	}
	public static int minimax(ParentBoard p,int cutoff, boolean maximizingPlayer)
	{
	int bestValue,val;
	if(p.depth == 0 && p.h == Integer.MIN_VALUE)
		print_log_mm.add("root"+","+p.depth+","+"-Infinity\n");
	else
	{
		if(p.h==Integer.MIN_VALUE)
			print_log_mm.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+"-Infinity\n");
		else if(p.h == Integer.MAX_VALUE)
			print_log_mm.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",Infinity\n");
		else
			print_log_mm.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+"\n");
	}
		if(p.depth == cut_off||p.order_children.size()==0)
		{
			return (p.h);	
		}
		if(maximizingPlayer)
		{
			
			bestValue = Integer.MIN_VALUE;	
			for(ParentBoard c: p.order_children)
			{
				 val = minimax(c,cutoff-1,false);
				bestValue = Math.max(bestValue, val);
				p.h = bestValue;
				if(p.depth ==0)
					print_log_mm.add("root"+","+p.depth+","+p.h+"\n");
				else
				{
					if(p.h == Integer.MIN_VALUE && p.depth!=0)
						print_log_mm.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",-Infinity\n");
					else
						print_log_mm.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+"\n");
				}
			}
			return bestValue;
		}
		else
		{
			
			bestValue = Integer.MAX_VALUE;

			for(ParentBoard c: p.order_children)
			{
				 val = minimax(c,cutoff-1, true);
				bestValue = Math.min(bestValue, val);
				p.h = bestValue;
				if(p.depth ==0)
					print_log_mm.add("root"+","+p.depth+","+p.h+"\n");
				else
				{
					if(p.h == Integer.MIN_VALUE)
						print_log_mm.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+",-Infinity\n");
					else
						print_log_mm.add(alphabetical_rc_ordering(p.rowP,p.colP)+","+p.depth+","+p.h+"\n");
				}
			}
			return bestValue;
		}
	}

	private static void get_children(ParentBoard p, String me,
			String him, int depth, int cut_off2) {
		
		if(depth!=cut_off)
		{ 
			if(depth==0)
			{
				create_call(me,him, p ,p.depth+1);
				
				get_children(p,me,him,p.depth+1,cut_off);
			}
			else if(depth>0)
			{
				synchronized(p.children)
				{
			for(Iterator<ParentBoard> c =  p.children.iterator();c.hasNext();)
			{
				ParentBoard child = c.next();
				if(depth%2!=0)
				{
					create_call(him, me,child,depth+1);					
				}
				if(depth%2==0)
				{
					create_call(me,him,child,depth+1);
					
				}
				
				get_children(child,me,him,child.depth+1,cut_off);
			}
				}
			}
		}	
		else return;
		
	}
	private static void create_call(String me, String him, ParentBoard p, int depth) {
		// TODO Auto-generated method stub
		create_moves(p,me,him,depth);
		
	}

	private static void greedy(String[][] board, String myPlayer, String opponent) throws IOException {
		

		File outFile = new File ("output.txt");
	    FileWriter fWriter = new FileWriter (outFile);
	    PrintWriter pw = new PrintWriter (fWriter);  
		int depth =1;
		ParentBoard parent_g = new ParentBoard();
		parent_g.arr = board;			//initialising board
		for(int i=0;i<8;i++)
		{
			for(int j=0;j<8;j++)
			{
				parent_g.arr[i][j] = board[i][j];
			}
		}
		create_moves(parent_g, myPlayer, opponent, depth);  //get the next move
		//calculate_heuristic(parent_g);
        
        //order children in increasing order of position and decreasing order of heuristic
		child_order_greedy(parent_g,parent_g.order_children);
        if(parent_g.order_children.size()==0)
		{
			for(int i=0;i<8;i++)
			{
				for(int j=0;j<8;j++)
				{
					pw.write(parent_g.arr[i][j]);
				}
				pw.write("");
			}
		}
		else
		{
		for(int i=0;i<8;i++)
		{
			for(int j=0;j<8;j++)
			{
				pw.write(parent_g.order_children.get(0).arr[i][j]);
			}
			pw.write("\n");
		}
		pw.flush();
		}

}
	public static void child_order_greedy(ParentBoard p, ArrayList<ParentBoard> order_children)
	{
			for(ParentBoard c: p.children)
			{
				order_children.add(c);
				Collections.sort(order_children, new Comparator<ParentBoard>(){
							public int compare(ParentBoard p1, ParentBoard p2){
								if(p1.h!=p2.h)
									return (p2.h - p1.h);
								if(p1.h == p2.h)
								{
									if(p1.rowP == p2.rowP)
									{
										return(p1.colP - p2.colP);
									}
									else
									{
										return(p1.rowP - p2.rowP);
									}

								}
								return 0;
								
							}
			});
				
			
		}
		
	}	


	//traverse in all 8 directions to find the opponent

	private static void create_moves(ParentBoard parent_g, String myPlayer,
			String opponent, int depth) {
		
			
		for(int i=0;i<8;i++)
		{
			for(int j=0;j<8;j++)
			{
				if(parent_g.arr[i][j].equals(myPlayer))
				{
					int row = i; 
					int col = j;
					if(parent_g.arr[row-1][col].equals(opponent))
					{	RowColMove rcm =traverse_north(row-1,col,parent_g,myPlayer,opponent, depth);
						if(rcm!=null)
							create_north(rcm,myPlayer,opponent,parent_g,row,col,depth);
					}
					if(parent_g.arr[row-1][col-1].equals(opponent))
					{
						RowColMove rcm = traverse_NW(row-1,col-1,parent_g,myPlayer);
						if(rcm!=null)create_NW(rcm,myPlayer,opponent,parent_g,row,col,depth);
					}
					if(parent_g.arr[row-1][col+1].equals(opponent)) // check for opponent on the right diagonal
					{
						RowColMove rcm = traverse_NE(row-1,col+1,parent_g,myPlayer);
						if(rcm!=null)create_NE(rcm,myPlayer,opponent,parent_g,row,col,depth);
					}
					if(parent_g.arr[row][col-1].equals(opponent))//check if opponent is present on the left of the player
					{
						
						RowColMove rcm = traverse_west(row,col-1,parent_g,myPlayer);
						if(rcm!=null)create_west(rcm,myPlayer,opponent,parent_g,row,col,depth);													
					} 
					if(parent_g.arr[row][col+1].equals(opponent)) //check if opponent is present on the right of the player
					{
						RowColMove rcm = traverse_east(row,col+1,parent_g,myPlayer);
						if(rcm!=null)	create_east(rcm,myPlayer,opponent,parent_g,row,col,depth);										
					}
					if(parent_g.arr[row+1][col].equals(opponent))//check if opponent is present below the player
					{			
						RowColMove rcm = traverse_south(row+1,col,parent_g,myPlayer);
						if(rcm!=null)create_south(rcm,myPlayer,opponent,parent_g,row,col,depth);											
					}
					//check for opponent on the right
								
					
					if(parent_g.arr[row+1][col-1].equals(opponent))
					{
						RowColMove rcm = traverse_SW(row+1,col-1,parent_g,myPlayer);
						if(rcm!=null)create_SW(rcm,myPlayer,opponent,parent_g,row,col,depth);
					}
					if(parent_g.arr[row+1][col+1].equals(opponent))
					{
						RowColMove rcm = traverse_SE(row+1,col+1,parent_g,myPlayer);
						if(rcm!=null)create_SE(rcm,myPlayer,opponent,parent_g,row,col,depth);
					}
					
				}
				
					}
			}
		
				
				}
	public static void create_SE(RowColMove rcm,String myPlayer, String opponent, ParentBoard parent, int row, int col,int depth) {
		// TODO Auto-generated method stub
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
	child[rcm.rowMove][rcm.colMove]=myPlayer;
	while(row!=rcm.rowMove && col!=rcm.colMove)
	{
		if(child[row][col].equals(opponent))
			child[row][col]=myPlayer;
					row++;
					col++;
	}
	ParentBoard child_add = new ParentBoard();
	create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove, depth);		
		
	}
	public static RowColMove traverse_SE(int r, int c, ParentBoard p,
			String myPlayer) {
		// TODO Auto-generated method stub
		for(int k =r,j=c;(k<=7)&&(j<=7);k++,j++) //traverse along SE
		{
			if(p.arr[k][j].equals(myPlayer))
				break;
			if(p.arr[k][j].equals("*")) 
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove = k;
				rcm.colMove = j;
				return rcm;
			}
			
		}
		return null;
	}

	public static void create_SW(RowColMove rcm,
			String myPlayer, String opponent, ParentBoard parent, int row, int col,int depth) {
		// TODO Auto-generated method stub
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
	child[rcm.rowMove][rcm.colMove]=myPlayer;
	while(row!=rcm.rowMove && col!=rcm.colMove)
	{
		if(child[row][col].equals(opponent))
			child[row][col]=myPlayer;
					row++;
					col--;
	}
	ParentBoard child_add = new ParentBoard();
	create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove, depth);			
	}

	public static RowColMove traverse_SW(int r, int c, ParentBoard p,
			String myPlayer) {
		// TODO Auto-generated method stub
		for(int k =r,j=c;(k<=7)&&(j>=0);k++,j--) //traverse along SW
		{
			if(p.arr[k][j].equals(myPlayer))
				break;
			if(p.arr[k][j].equals("*"))
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove = k;
				rcm.colMove = j;
				return rcm;
			}
			
		}
		return null;
	}

	public static RowColMove traverse_south(int r, int c, ParentBoard p,
			String myPlayer) {
		// TODO Auto-generated method stub
		for(int j=r;j<=7;j++)
		{
			if(p.arr[j][c].equals(myPlayer))
				break;
			if(p.arr[j][c].equals("*"))
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove = j;
				rcm.colMove = c;
				return rcm;
			}
		}
		
		return null;
	}
	public static void create_south(RowColMove rcm,
			String myPlayer, String opponent, ParentBoard parent, int r, int c, int depth) {
		// TODO Auto-generated method stub
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
	child[rcm.rowMove][rcm.colMove]=myPlayer;
	while(r!=rcm.rowMove)
	{
		if(child[r][c].equals(opponent))
			child[r][c]=myPlayer;
					r++;
	}
	ParentBoard child_add = new ParentBoard();
	create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove, depth);	
		
	}
	public static void create_east(RowColMove rcm,
			String myPlayer, String opponent, ParentBoard parent, int r, int c, int depth) {
		// TODO Auto-generated method stub
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
	child[rcm.rowMove][rcm.colMove]=myPlayer;
	while(c!=rcm.colMove)
	{
		if(child[r][c].equals(opponent))
			child[r][c]=myPlayer;
					c++;
	}
	ParentBoard child_add = new ParentBoard();
	create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove,depth);	
	}
	public static RowColMove traverse_east(int r, int c, ParentBoard p,
			String myPlayer) {
		// TODO Auto-generated method stub
		for(int j=c;j<=7;j++)
		{// traverse E
			
			if(p.arr[r][j].equals("*"))
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove=r;
				rcm.colMove=j;
				return rcm;
			
			}
		}
		return null;
	}

	public static void create_west(RowColMove rcm,
			String myPlayer, String opponent, ParentBoard parent, int r, int c, int depth) {
		// TODO Auto-generated method stub
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
		child[rcm.rowMove][rcm.colMove]=myPlayer;
		while(c!=rcm.colMove)
		{
			if(child[r][c].equals(opponent))
				child[r][c]=myPlayer;
						c--;
		}
		ParentBoard child_add = new ParentBoard();
		create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove, depth);	
	
	
		
	}
	public static RowColMove traverse_west(int r, int c, ParentBoard p,
			String myPlayer) {
		// TODO Auto-generated method stub
		 // traverse W
		for(int j=c;j>=0;j--)
		{
			if(p.arr[r][j].equals(myPlayer))
				break;
			if(p.arr[r][j].equals("*"))
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove=r;
				rcm.colMove=j;
				return rcm;
			}
		}
		return null;
	}
	public static void create_NE(RowColMove rcm,
			String myPlayer, String opponent, ParentBoard parent, int row, int col,int depth) {
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
	child[rcm.rowMove][rcm.colMove]=myPlayer;
	while(row!=rcm.rowMove && col!=rcm.colMove)
	{
		if(child[row][col].equals(opponent))
			child[row][col]=myPlayer;
					row--;
					col++;
	}
	ParentBoard child_add = new ParentBoard();
	create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove,depth);	
		
	}

	public static RowColMove traverse_NE(int r, int c, ParentBoard p,
			String myPlayer) {
		for(int k=r,j=c;(k>=0)&&(j<=7);k--,j++)
		{
			if(p.arr[k][j].equals(myPlayer))
				break;
			if(p.arr[k][j].equals("*"))
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove = k;
				rcm.colMove = j;
				return rcm;
			}
		}
		return null;
	}

	public static RowColMove traverse_NW(int r, int c, ParentBoard p,
			String myPlayer) {
		// TODO Auto-generated method stub
		for(int k =r,j=c;(k>=0)&&(j>=0);k--,j--) //traverse along NW
		{
			if(p.arr[k][j].equals(myPlayer))
				break;
			if(p.arr[k][j].equals("*"))
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove = k;
				rcm.colMove = j;
				return rcm;
			}
			
		}
		return null;
	}
	public static void create_NW(RowColMove rcm,
			String myPlayer, String opponent, ParentBoard parent, int row, int col, int depth) {
		// TODO Auto-generated method stub
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
	child[rcm.rowMove][rcm.colMove]=myPlayer;
	while(row!=rcm.rowMove && col!=rcm.colMove)
	{
		if(child[row][col].equals(opponent))
			child[row][col]=myPlayer;
					row--;
					col--;
	}
	ParentBoard child_add = new ParentBoard();
	create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove,depth);		
		
	}
	private static RowColMove traverse_north(int r, int c, ParentBoard p,
			String myPlayer, String opponent, int depth) {
		for(int j=r-1;j>=0;j--)
		{
			if(p.arr[j][c].equals(myPlayer))
				break;
			if(p.arr[j][c].equals("*"))
			{
				RowColMove rcm = new RowColMove();
				rcm.rowMove = j;
				rcm.colMove = c;
				return rcm;
			}
		
	}
		return null;

	}

	public static void create_north(RowColMove rcm,String myPlayer,String opponent, ParentBoard parent,int r,int c, int depth) {
		// TODO Auto-generated method stub
		String child[][] = new String[8][8];
		for(int i=0;i<8;i++)
			for(int j=0;j<8;j++)
				child[i][j]=parent.arr[i][j];
		child[rcm.rowMove][rcm.colMove]=myPlayer;
		
			
	
	while(r!=rcm.rowMove)
	{
		
		if(child[r][c].equals(opponent))
			child[r][c]=myPlayer;
		r--;
	}
	ParentBoard child_add = new ParentBoard();	
	create_child(child_add,child,parent,myPlayer,opponent,rcm.rowMove,rcm.colMove,depth);
		
		
	}
private static void create_child(ParentBoard child_add, String[][] child,
		ParentBoard parent, String myPlayer, String opponent, int rowP, int colP,
		int depth) {

	for(int a=0;a<8;a++)
	{
		for(int b=0;b<8;b++)
		{
			child_add.arr[a][b]=child[a][b];			
		}
	}	
	child_add.depth = depth;	
	child_add.rowP = rowP;
	child_add.colP = colP;
	parent.children.add(child_add);	
	if(task == 2 && child_add.depth%2==0)
		child_add.h = Integer.MIN_VALUE;
	if(task ==2 && child_add.depth%2!=0)
		child_add.h = Integer.MAX_VALUE;
	if(task == 3 && child_add.depth%2==0)
		child_add.h = Integer.MIN_VALUE;
	if(task ==3 && child_add.depth%2!=0)
		child_add.h = Integer.MAX_VALUE;
	if(task == 3 && child_add.depth == cut_off)
		calculate_heuristic(child_add);
	if(task == 2 && child_add.depth == cut_off)
		calculate_heuristic(child_add);
	if(task == 1)
		calculate_heuristic(child_add);
	
	
}

private static void calculate_heuristic(ParentBoard c) {
	
	{
		int myH =0,oH =0;
		for(int i=0;i<8;i++)
		{
			for(int j=0;j<8;j++)
			{
				if(c.arr[i][j].equals(maxPlayer))
					myH = myH+pos_matrix[i][j];
				if(c.arr[i][j].equals(minPlayer))
					oH = oH+pos_matrix[i][j];
			}
		}
		int h = myH - oH;
		c.h = h;
	}
	

}}
class ParentBoard
{
	String[][] arr;
	int rowP; int oW,myW;
	int colP; int same_h_r; int same_h_c;
	int depth;
	int h;
	List<ParentBoard> children= new CopyOnWriteArrayList <ParentBoard>();
	ArrayList<ParentBoard> order_children = new ArrayList<ParentBoard>();
	public ParentBoard()
	{
		arr=new String[8][8];
			
	}
	
}
class RowColMove
{
	int rowMove;
	int colMove;
}
