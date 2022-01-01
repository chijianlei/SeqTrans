package cost_matrix;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import gumtreediff.tree.TreeUtils;

public class Matrix {
	private static double edgenum1;
	private static double edgenum2;
	private static TreeContext tContext1;
	private static TreeContext tContext2;
	private static int[][] graph1;
	private static int[][] graph2;
	private static int[][] costmatrix;
	private static ArrayList<Node> nodes1= new ArrayList<>();
	private static ArrayList<Node> nodes2= new ArrayList<>();
	private static ArrayList<Edge> edges1 = new ArrayList<>();
	private static ArrayList<Edge> edges2 = new ArrayList<>();
	private static Map<ITree, Integer> node2num1 = new HashMap<>();
	private static Map<ITree, Integer> node2num2 = new HashMap<>();
	private static Map<Integer, Integer> map = new HashMap<>();
	private static Map<Integer, Integer> change = new HashMap<>();

	public static void main(String[] args) throws Exception{
		String path = "talker.cpp";
		File cppfile = new File(path);
		TreeContext tc1 = new SrcmlCppTreeGenerator().generateFromFile(cppfile);
		ITree root1 = tc1.getRoot();
		String path2 = "talker2.cpp";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlCppTreeGenerator().generateFromFile(cppfile2);
		ITree root2 = tc2.getRoot();
		double sim = GetSimilarity(root1, root2, tc1, tc2);
		System.out.println("Smilarity:"+sim);
   }


	public static double GetSimilarity(ITree root1, ITree root2, TreeContext tc1, TreeContext tc2){
		tContext1 = tc1;
		tContext2 = tc2;
		List<ITree> list1 = TreeUtils.preOrder(root1);
		List<ITree> list2 = TreeUtils.preOrder(root2);
		nodes1 = new ArrayList<>();
		nodes2 = new ArrayList<>();
		ReadNode(list1, list2);
		int m = nodes1.size();
		System.out.println("m:"+m);//测试节点总数
		int n = nodes2.size();
		System.out.println("n:"+n);
		compare(nodes1, nodes2);
		edges1 = new ArrayList<>();
		edges2 = new ArrayList<>();
		graph1 = new int[m][m];
		graph2 = new int[n][n];
		ReadEdge(list1, list2);
//		System.out.println(graph1[0][2]);//测试边读入情况
		Traversal(nodes1,graph1);
		Traversal(nodes2,graph2);
		CreatCostmatrix(graph1,graph2,nodes1,nodes2);
//		String matrixpath = "matrix.txt";
//		for(int i =0;i<m+n;i++){
//			for(int j =0;j<m+n;j++){
//				String content = String.valueOf(costmatrix[i][j]);
////				System.out.println(costmatrix[i][j]);//测试消耗矩阵读入情况
//				Writetxt(matrixpath,content+",");
//			}
//			Writetxt(matrixpath,"\r\n");
//		}
		System.out.println("Hungry遍历中");
		KuhnMunkres km = new KuhnMunkres(m+n);
		double[] result = new double[m+n];
		int[][] recostmatrix = new int[m+n][m+n];
		for(int i=0;i<m+n;i++){
			for(int j=0;j<m+n;j++){
				recostmatrix[i][j] = -costmatrix[i][j];
			}
		}
		int[][] re = KuhnMunkres.getMaxBipartie(recostmatrix,result);//KuhnMunkres算法计算最小代价的二分匹配
		int len = Math.min(km.getlenX(), km.getlenY());
		System.out.println("Hungry遍历完毕"+" len:"+len);
		map = new HashMap<>();
		for(int i=0;i<len;i++){
			map.put(re[i][0],re[i][1]);
		}
		change = new HashMap<>();
		Iterator iter = map.entrySet().iterator();
		while (iter.hasNext()) {
			Map.Entry entry = (Map.Entry) iter.next();
			Object key = entry.getKey();
			Object val = entry.getValue();
			int i = map.get(key).intValue();
			int j = map.get(val).intValue();
			if(i<m)
				change.put(i, j);//构建匹配矩阵
//			System.out.println(i + " " + j);//测试二分匹配序列
		}
		Iterator iter1 = change.entrySet().iterator();
		while (iter1.hasNext()) {
			Map.Entry entry = (Map.Entry) iter1.next();
			Object key = entry.getKey();
			Object val = entry.getValue();
			System.out.println(key + " " + val);//测试匹配矩阵
		}
//		for(Edge edge : edges1){
//			int source = edge.getSource();
//			int target = edge.getTarget();
//			int weight = edge.getweight();
//			System.out.println(source + " " + target+ " " +weight);//测试边链表
//		}
//		System.out.println(nodes1.get(0).getName());
//		System.out.println(nodes2.get(0).getName());
//		Boolean shiBoolean = nodes1.get(0).getName().equals(nodes2.get(0).getName());
//		System.out.println(shiBoolean);
		long NodeCost = NodeCost(map,m,n);
		long EdgeCost = EdgeCost(map,m,n);
		System.out.println("NodeCost:"+NodeCost);
		System.out.println("EdgeCost:"+EdgeCost);
		long EditDistance = NodeCost + EdgeCost;//图编辑距离
		double Similarity = EditDistance/(m+n+edgenum1+edgenum2);
		System.out.println("EditDistance:"+EditDistance);//测试图编辑距离
		System.out.println("Similarity:"+Similarity);//测试图相似度
//		Map<Integer, Integer> map1 = nodes1.get(0).getinMap();
//		Map<Integer, Integer> map2 = nodes2.get(0).getinMap();
//		ArrayList<Integer> integer = new ArrayList<Integer>();
//		System.out.println(Aggregate.Travel(map1));
//		System.out.println(Aggregate.Travel(map2));//测试0号出边数组
//		integer = Aggregate.intersect(map1, map2);
//		System.out.println(integer);//测试0号出边交集
//		Map<Integer, Integer> test1 = nodes1.get(0).getoutMap();
//		Iterator iter = test1.entrySet().iterator();
//		while (iter.hasNext()) {
//			Map.Entry entry = (Map.Entry) iter.next();
//			Object key = entry.getKey();
//			Object val = entry.getValue();
//			System.out.println(key + " " + val);//测试0号节点出度序列
//		}
		return Similarity;
	}

	public static long NodeCost(Map<Integer, Integer> map,int m,int n){
		long nodecost = 0;
		Iterator iter = map.entrySet().iterator();
		while (iter.hasNext()) {
			Map.Entry entry = (Map.Entry) iter.next();
			Integer key = (Integer)entry.getKey();
			Integer val = (Integer)entry.getValue();
			int i = key.intValue();
			int j = val.intValue();
//			System.out.println("Node:"+i + " " + j);
			if(i<m && j<n){
				if(!(nodes1.get(i).getName().equals(nodes2.get(j).getName()))){
//					System.out.println("Node:"+i + " " + j);
//					System.out.println("Node:"+nodes1.get(i).getName() + " " + nodes2.get(j).getName());
					nodecost = nodecost+1;//重命名代价

				}
			}
			else if(i<m && j>=n && j<m+n){
				nodecost = nodecost+1;

			}
			else if(i>=m && i<m+n && j<n){
				nodecost = nodecost+1;//空节点匹配真实节点cost+1

			}

		}
//		System.out.println("count:"+ count);
		return nodecost;
	}

	public static long EdgeCost(Map<Integer, Integer> map,int m,int n){
		int count = 0;
		double edgecost = 0;
		int source1 = 0;
		int source2 = 0;
		int target1 = 0;
		int target2 = 0;
		double weight = 0;
		int weight1 = 0;
		int weight2 = 0;
		int change1 = 0;
		int change2 = 0;
		for(Edge edge1 : edges1){
			source1 = edge1.getSource();
			target1 = edge1.getTarget();
			weight1 = edge1.getweight();
			change1 = change.get(Integer.valueOf(source1)).intValue();
			change2 = change.get(Integer.valueOf(target1)).intValue();
//			System.out.println("change1:"+ change1);
//			System.out.println("change2:"+ change2);
			for(Edge edge2 : edges2){
				source2 = edge2.getSource();
				target2 = edge2.getTarget();
				if(change1 == source2 && change2 == target2){
//					count++;//不考虑权重使用count
//					System.out.println(source1+">"+source2+" "+target1+">"+target2);//查看匹配序列
					weight2 = edge2.getweight();
					weight = Math.min(weight1, weight2);
					edgecost = edgecost+weight;//是否考虑权重?
//					System.out.println("weight:"+ weight);
				}
			}
		}
//		edgecost = count;
		System.out.println("count:"+ count);
//		System.out.println("edgecost:"+ edgecost);
		edgecost = edgenum1+edgenum2-2*edgecost;
		return (long)edgecost;
	}


	 public static int[][] CreatCostmatrix(int[][] graph1,int[][] graph2,ArrayList<Node> nodes1,ArrayList<Node> nodes2){
		 System.out.println("创建消耗矩阵中");
		 int m = nodes1.size();
		 int n = nodes2.size();
		 double weight_out1;
		 double weight_out2;
		 Map<Integer, Double> outmap1 = new HashMap<>();
		 Map<Integer, Double> outmap2 = new HashMap<>();
		 costmatrix = new int[m+n][m+n];
		 int outedge_cost = 0;
		 int relabel_cost = 0;
		 for(int i = 0;i < m;i++){
			 outmap1 = nodes1.get(i).getoutMap();
//			 for(int in : inset1){
//				 System.out.println("inedge1:"+in);
//			 }
			 weight_out1 = Aggregate.Travel(outmap1);
			 for(int j = 0;j < n;j++){
				 outmap2 = nodes2.get(j).getoutMap();
//				 for(int in : inset2){
//					 System.out.println("inedge2:"+in);
//				 }
				 weight_out2 = Aggregate.Travel(outmap2);
				 if(!(nodes1.get(i).getName().equals(nodes2.get(j).getName())))
					 relabel_cost = 1;
				 else relabel_cost = 0;
				 outedge_cost = (int) Math.abs(weight_out1-weight_out2);
//				 System.out.println("["+i+"]["+j+"]:");
				 System.out.println("outedge_cost:"+outedge_cost);
				 costmatrix[i][j] = relabel_cost+outedge_cost;//左上角矩阵cost
			 }
		 }
//		 System.out.println("costmatrix:"+costmatrix[1][1]);
		 for(int i = 0;i<m;i++){
			 outmap1 = nodes1.get(i).getoutMap();
			 weight_out1 = Aggregate.Travel(outmap1);
			 outedge_cost = (int) weight_out1;
			 for(int j = n;j<m+n;j++){
				 costmatrix[i][j] = 0x7fffffff;//设置非对角线消耗为无穷大
			 }
			 costmatrix[i][n+i] = 1+ outedge_cost;//左下角矩阵
		 }
		 for(int j = 0;j<n;j++){
			 outmap2 = nodes2.get(j).getoutMap();
			 weight_out2 = Aggregate.Travel(outmap2);
			 outedge_cost = (int) weight_out2;
			 for(int i = m;i<m+n;i++){
				 costmatrix[i][j] = 0x7fffffff;
			 }
			 costmatrix[m+j][j] = 1+ outedge_cost;//右上角矩阵
		 }//右下角矩阵为空
		 return costmatrix;
	 }

	 public static void compare(ArrayList<Node> nodes1,ArrayList<Node> nodes2){
		 int m = nodes1.size();
		 int n = nodes2.size();
		 for(int i=0;i<m;i++){
			 String name1 = nodes1.get(i).getName();
			 for(int j=0;j<n;j++){
				 String name2 = nodes2.get(j).getName();
				 if(name1.equals(name2))
					 break;
				 else if(j == n-1)
					 System.out.println("name:"+name1);
			 }
		 }
	 }

	 public static void Traversal(ArrayList<Node> nodes,int[][] graph){
		 System.out.println("遍历中");
		 int size = nodes.size();
		 for(int i = 0;i < size;i++){
			 for(int j = 0;j<size;j++){
//				if(graph[j][i]!=0){
//					Node in =  nodes.get(j);
//					int innum = nodes.get(j).getNum();
////					System.out.println(innum);//检查入边数组
//					nodes.get(i).setParent(in);
//				    nodes.get(i).setinMap(innum, graph[j][i]);//不考虑权重就设为1
//				}
				if(graph[i][j]!=0){
					Node out =  nodes.get(j);
					int outnum = nodes.get(j).getNum();
//					System.out.println(outnum);//检查出边数组
					nodes.get(i).setChild(out);
					nodes.get(i).setoutMap(outnum, graph[i][j]);//不考虑权重就设为1
				}
			 }
		 }

	 }//图遍历写入边出边

	 public static void ReadEdge(List<ITree> list1, List<ITree> list2){
		 edgenum1 = 0;
		 edgenum2 = 0;
		 for(ITree t : list1) {
			 List<ITree> children = t.getChildren();
			 int src = node2num1.get(t);
			 if(children.size()!=0) {
				 for(ITree child : children) {
					 Edge edge = new Edge();
					 int dst = node2num1.get(child);
					 edge.setSource(src);
					 edge.setTarget(dst);
					 edge.setweight(1);//weight is 1 now as a temp.
					 edges1.add(edge);
					 System.out.println(src+"->"+dst);
					 graph1[src][dst] = 1;
					 edgenum1 = edgenum1+1;//是否考虑权重
				 }
			 }
		 }

		 for(ITree t : list2) {
			 List<ITree> children = t.getChildren();
			 int src = node2num2.get(t);
			 if(children.size()!=0) {
				 for(ITree child : children) {
					 Edge edge = new Edge();
					 int dst = node2num2.get(child);
					 edge.setSource(src);
					 edge.setTarget(dst);
					 edge.setweight(1);//weight is 1 now as a temp.
					 edges2.add(edge);
					 System.out.println(src+"->"+dst);
					 graph2[src][dst] = 1;
					 edgenum2 = edgenum2+1;//是否考虑权重
				 }
			 }
		 }
	 }

     public static void ReadNode(List<ITree> list1, List<ITree> list2){
	   for(int i=0;i<list1.size();i++) {
		   ITree t = list1.get(i);
		   Node node = new Node();
		   int num = i;
	       int typeNum = t.getType();
	       String attr = tContext1.getTypeLabel(t);
	       node.setNum(num);
	       node.setTypeNume(typeNum);
	       node.setName(attr);
	       nodes1.add(node);
	       node2num1.put(t, num);
	   }

	   for(int i=0;i<list2.size();i++) {
		   ITree t = list2.get(i);
		   Node node = new Node();
		   int num = i;
	       int typeNum = t.getType();
	       String attr = tContext2.getTypeLabel(t);
	       node.setNum(num);
	       node.setTypeNume(typeNum);
	       node.setName(attr);
	       nodes2.add(node);
	       node2num2.put(t, num);
	   }
   }

     public static ArrayList<File> getFiles(String path){
 		ArrayList<File> fileList=new ArrayList<>();
 		File fpath=new File(path);
 		if(fpath.exists()){
 			File[] files=fpath.listFiles();
 			for (File file : files) {
 				if(file.isFile()&&file.getName().contains("dat")){
 					fileList.add(file);
 				}
 			}
 		}
 		return fileList;
 	}

 	public static void Writetxt(String path,String Content) {
        FileWriter fw = null;
        try {
            fw = new FileWriter(path,true);
            fw.write(Content);
            fw.flush();
            fw.close();
        } catch (IOException e1) {
            e1.printStackTrace();
            System.out.println("写入失败");
            System.exit(-1);
        }
    }
}
