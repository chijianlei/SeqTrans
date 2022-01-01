package structure;

import java.util.List;

import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class Location {

	private int BeginLine;
	private int LastLine;
	private int BeginCol;
	private int LastCol;
	
	public Location(SubTree st) {
		TreeContext tc = st.getTC();
		ITree stRoot = st.getRoot();
		List<ITree> nodesList = stRoot.getDescendants();
		nodesList.add(stRoot);//srcT所有节点
		int BeginLine = 0;
		int LastLine = 0;
		int BeginCol = 0;
		int LastCol = 0;
		for(ITree node : nodesList) {
			int line = node.getLine();
			int col = node.getColumn();
			int lastLine = node.getLastLine();
			int lastCol = node.getLastColumn();
			String type = tc.getTypeLabel(node);
			if(!type.equals("block")) {//跳过block节点，该节点会导致lastline为大括号结束位置
				if(BeginLine==0&&line!=0) {
					BeginLine = line;
				}else if(line < BeginLine&&line!=0) {
					BeginLine = line;
				}//begin line
				if(BeginCol==0&&col!=0) {
					BeginCol = col;
				}else if(col < BeginCol&&col!=0) {
					BeginCol = col;
				}//begin column
				if(lastLine > LastLine) {
					LastLine = lastLine;
				}//last line
				if(lastCol > LastCol) {
					LastCol = lastCol;
				}//last column
			}else if(type.equals("empty_stmt"))//特殊情况	
				continue;
		}	
		this.BeginLine = BeginLine;
		this.BeginCol = BeginCol;
		this.LastLine = LastLine;
		this.LastCol = LastCol;
		System.out.println("location:"+BeginLine+","+LastLine+","+BeginCol+","+LastCol);
	}

	public int getBeginLine() {
		return BeginLine;
	}

	public int getLastLine() {
		return LastLine;
	}

	public int getBeginCol() {
		return BeginCol;
	}

	public int getLastCol() {
		return LastCol;
	}

	
	
}
