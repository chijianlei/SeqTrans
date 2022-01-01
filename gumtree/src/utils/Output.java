package utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Pattern;

import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.io.TreeIoUtils;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import gumtreediff.tree.TreeUtils;
import split.Split;
import structure.Definition;
import structure.Migration;
import structure.SubTree;
import structure.Transform;

public class Output {

	public static void main (String args[]) throws Exception{
		String path = "migrations_test";
//		String path = "Absolute3DLocalizationElement.cpp";
//		Output.collectTokens(sp.trans);
//		Output.collectChangePairs(path, "");
//		Output.collectDefUse(path, "", "");
		Output.printJson(path, null);
//		String path = "talker.cpp";
//		Output.tokensFromInputFile(path);
	}

	public static void tokensFromInputFile(String path) throws Exception {
		Split sp = new Split();
		File output = new File("src-test.txt");
		File outLine = new File("lineNum.txt");
		File outFile = new File("src-var.txt");
		BufferedWriter wr = new BufferedWriter(new FileWriter(output));
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(outLine));
		BufferedWriter wr2 = new BufferedWriter(new FileWriter(outFile));
		File cppFile = new File(path);
		TreeContext tc = new SrcmlCppTreeGenerator().generateFromFile(cppFile);
		String cppName = cppFile.getName();
		System.out.println("Reading file: "+cppName);
		Defuse def = new Defuse();
		ArrayList<Definition> defs1 = def.getDef(tc, "src");//先计算action,再收集defs
        HashMap<String, ArrayList<Definition>> defMap1 = def.transferDefs(defs1);
        HashMap<ITree, ArrayList<Definition>> blockMap1 = def.transferBlockMap(defs1, tc, "src");
		ArrayList<SubTree> sub1 = sp.splitSubTree(tc, cppName);//Subtree中割裂过block,注意
		for(SubTree srcT : sub1) {
			ITree root = srcT.getRoot();
			List<ITree> candidates = root.getDescendants();
			if(srcT!=null&&srcT.getTC()!=null) {
				String src = subtree2src(srcT);

				ArrayList<ITree> leaves1 = new ArrayList<>();
				Utils.traverse2Leaf(root, leaves1);
				int labelCount = 0;
				for(ITree leaf : leaves1) {
					String label = leaf.getLabel();
					if(!label.equals(""))
						labelCount++;
					String type = tc.getTypeLabel(leaf);
					if(type.equals("literal")) {
						leaf.setLabel(Output.deleteLiteral(leaf, tc));
					}
					ArrayList<Definition> stringList = defMap1.get(label);
					if(stringList!=null) {
						ITree parBlock = def.searchBlock(leaf, tc);
						ArrayList<Definition> blockList = blockMap1.get(parBlock);
						for(Definition def1 : stringList) {
							if(blockList!=null) {
								if(blockList.contains(def1)) {
									if(leaf.getId()>def1.getDefLabelID()) {
										leaf.setLabel("var");
									}
								}
							}
							if(def1.getDefLabelID()==leaf.getId()) {
								leaf.setLabel("var");
							}
						}
					}
				}
				if(labelCount<=1)
					continue;

				wr.append(src);
				wr.newLine();
				wr.flush();
				src = subtree2src(srcT);
				wr2.append(src);
				wr2.newLine();
				wr2.flush();

				candidates.add(root);
				int beginLine = 0;
				int beginColumn = 0;
				int endLine = 0;
				int endColumn = 0;
				for(ITree node : candidates) {
					int line = node.getLine();
					int column = node.getColumn();
					if(line==0)
						continue;//null
					int lastLine = node.getLastLine();
					int lastColumn = node.getLastColumn();
					if(beginLine==0&&line>beginLine) {
						beginLine = line;
					}else if(line<beginLine&&line!=0){
						beginLine = line;
					}
					if(beginColumn==0&&column>beginColumn) {
						beginColumn = column;
					}else if(column<beginColumn&&column!=0){
						beginColumn = column;
					}
					if(endLine==0&&lastLine>endLine) {
						endLine = lastLine;
					}else if(lastLine>endLine) {
						endLine = lastLine;
					}
					if(endColumn==0&&lastColumn>endColumn) {
						endColumn = lastColumn;
					}else if(lastColumn>endColumn) {
						endColumn = lastColumn;
					}
				}
				wr1.append(String.valueOf(beginLine)+","+String.valueOf(beginColumn)+
						"->"+String.valueOf(endLine)+","+String.valueOf(endColumn));
				wr1.newLine();
				wr1.flush();
			}
		}
		wr.close();
		wr1.close();
		wr2.close();
	}

	public static void printJson(String path, String filter) throws Exception {
		Split sp = new Split();
		ArrayList<Migration> migrats = new ArrayList<>();
		migrats = sp.readMigration(path, filter);
		sp.storeTrans(migrats);
		ArrayList<Transform> trans = sp.trans;
		for(int i=0;i<trans.size();i++) {
			Transform tf = trans.get(i);
			System.out.println("Analyse:"+tf.getMiName());
			SubTree srcT = tf.getSTree();
			SubTree dstT = tf.getDTree();
			if(srcT!=null&&dstT!=null) {
                if(srcT.getTC()!=null&&dstT.getTC()!=null) {
                	TreeContext sub1 = new TreeContext();
        			TreeContext sub2 = new TreeContext();
        			sub1.importTypeLabels(srcT.getTC());
        			sub2.importTypeLabels(dstT.getTC());
        			sub1.setRoot(srcT.getRoot());
        			sub2.setRoot(dstT.getRoot());
        			String out = "jsons\\pair"+String.valueOf(i)+"_src.json";
        			BufferedWriter wr = new BufferedWriter(new FileWriter(new File(out)));
        			wr.append(TreeIoUtils.toJson(sub1).toString());
        			wr.flush();
        			wr.close();
        			String out1 = "jsons\\pair"+String.valueOf(i)+"_tgt.json";
        			BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File(out1)));
        			wr1.append(TreeIoUtils.toJson(sub2).toString());
        			wr1.flush();
        			wr1.close();
				}
			}
		}
	}

	public static void printJson1(String path, String filter) throws Exception {
		Split sp = new Split();
		ArrayList<Migration> migrats = new ArrayList<>();
		migrats = sp.readMigration(path, filter);
		for(int i=0;i<migrats.size();i++) {
			Migration mi = migrats.get(i);
			TreeContext srcT = mi.getSrcT();
			TreeContext dstT = mi.getDstT();
			if(srcT!=null&&dstT!=null) {
				String out = "jsons\\pair"+String.valueOf(i)+"_src.json";
    			BufferedWriter wr = new BufferedWriter(new FileWriter(new File(out)));
    			wr.append(TreeIoUtils.toJson(srcT).toString());
    			wr.flush();
    			wr.close();
    			String out1 = "jsons\\pair"+String.valueOf(i)+"_tgt.json";
    			BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File(out1)));
    			wr1.append(TreeIoUtils.toJson(dstT).toString());
    			wr1.flush();
    			wr1.close();
			}
		}
	}

	public static void collectChangePairs(String path, String filter) throws Exception {
		Split sp = new Split();
		ArrayList<Migration> migrats = new ArrayList<>();
		migrats = sp.readMigration(path, filter);
		sp.storeTrans(migrats);
		String out = "change.txt";
		String out1 = "src-train.txt";
		String out2 = "tgt-train.txt";
		BufferedWriter wr = new BufferedWriter(new FileWriter(new File(out)));
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File(out1)));
		BufferedWriter wr2 = new BufferedWriter(new FileWriter(new File(out2)));
		for(Transform tf : sp.trans) {
			System.out.println("===============================");
			SubTree srcT = tf.getSTree();
			SubTree dstT = tf.getDTree();
			if(srcT!=null&&dstT!=null) {
				if(srcT.getTC()!=null&&dstT.getTC()!=null) {
					String src = subtree2src(srcT);
					String dst = subtree2src(dstT);
					if(!(src.contains("error situation")||dst.contains("error situation"))) {
//						System.out.println(src);
						wr.append(src+"\t");
						wr1.append(src);
						wr1.newLine();
						wr1.flush();
//						System.out.println(dst);
						wr.append(dst);
						wr.newLine();
						wr.flush();
						wr2.append(dst);
						wr2.newLine();
						wr2.flush();
					}
				}
			}
		}
		wr.close();
		wr1.close();
		wr2.close();
	}

	public static void collectTokens(ArrayList<Transform> trans) throws Exception {
		String out1 = "srcDiffToken.txt";
		String out2 = "dstDiffToken.txt";
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File(out1)));
		BufferedWriter wr2 = new BufferedWriter(new FileWriter(new File(out2)));
		for(Transform tf : trans) {
			System.out.println("===============================");
			SubTree srcT = tf.getSTree();
			SubTree dstT = tf.getDTree();
			if(srcT!=null&&dstT!=null) {
				List<ITree> leaves1 = new ArrayList<>();
				List<ITree> leaves2 = new ArrayList<>();
				if(srcT.getTC()!=null&&dstT.getTC()!=null) {
					leaves1 = Utils.traverse2Leaf(srcT.getRoot(), leaves1);
					leaves2 = Utils.traverse2Leaf(dstT.getRoot(), leaves2);
					for(int i=0;i<leaves1.size();i++) {
						ITree leaf = leaves1.get(i);
						if(leaf.getLabel()!=null) {
							wr1.append(leaf.getLabel());
							if(i!=leaves1.size()-1)
								wr1.append(" ");
						}
					}
					wr1.append("\t");
//					wr1.newLine();
//					wr1.flush();
					for(int i=0;i<leaves2.size();i++) {
						ITree leaf = leaves2.get(i);
						if(leaf.getLabel()!=null) {
							wr1.append(leaf.getLabel());
							if(i!=leaves2.size()-1)
								wr1.append(" ");
						}
					}
					wr1.newLine();
					wr1.flush();
//					wr2.append(s2);
//					wr2.newLine();
//					wr2.flush();
				}
			}
		}
		wr1.close();
		wr2.close();
	}

	public static ArrayList<Integer> locateLineNum(SubTree st, String path) throws Exception {
		ArrayList<Integer> candidates = new ArrayList<>();
		List<ITree> leaves = new ArrayList<>();
		leaves = Utils.traverse2Leaf(st.getRoot(), leaves);
//		System.out.println("tokens:"+Utils.printToken(st));
		ArrayList<String> labels = new ArrayList<>();
		ArrayList<String> codes = new ArrayList<>();
		for(ITree tmp : leaves) {
			String label = tmp.getLabel();
			labels.add(label);
		}
		File file = new File(path);
		BufferedReader br = new BufferedReader(new FileReader(file));
		String tmpline = "";
		while((tmpline = br.readLine())!=null) {
			codes.add(tmpline);
		}
		br.close();
		float sim = Float.MIN_VALUE;
		for(String code : codes) {
			int count = 0;
			for(String label : labels) {
				if(code.contains(label)) {
					count++;
				}
			}
			float tmpsim = (float)count/(float)(labels.size());
			if(tmpsim>sim) {
				candidates.clear();
				sim = tmpsim;
				int lineNum = codes.indexOf(code)+1;
				candidates.add(lineNum);
			}else if(tmpsim==sim) {
				int lineNum = codes.indexOf(code)+1;
				candidates.add(lineNum);
			}
		}
		return candidates;
	}//search整份代码，找到包含最多tokens的行号(可能不唯一)

	public static String subtree2src(SubTree st) throws Exception {
//		String src = String.valueOf(st.getRoot().getId());
		String src = "";
		String loopEnd = "";
		ITree root = st.getRoot();
		TreeContext srcT = st.getTC();
		String sType = srcT.getTypeLabel(root);
		if(sType.equals("while")||sType.equals("for")||sType.equals("if")) {
			if(sType.equals("while"))
				src = src+"while ( ";
			if(sType.equals("for"))
				src = src+"for ( ";
			if(sType.equals("if"))
				src = src+"if ( ";
			loopEnd = " ) ";
		}else if (sType.equals("return")){
			src = src+"return ";
		}

		List<ITree> leaves = new ArrayList<>();
		leaves = Utils.traverse2Leaf(root, leaves);
//		System.out.println(leaves.size());
//		for(ITree leaf : leaves) {
//			String type = srcT.getTypeLabel(leaf);
//			String label = leaf.getLabel();
//			System.out.println(type+":"+label);
//		}
		if(leaves.size()==0)
			throw new Exception("null leaves");
		else if(leaves.size()==1) {
			src = src+leaves.get(0).getLabel();//先把0号叶子放入
			return src;
		}

		src = src+leaves.get(0).getLabel();//先把0号叶子放入
//		System.out.println("leafSize:"+leaves.size());
		for(int i=0;i<leaves.size()-1;i++) {
//			System.out.println(src);
			int size = 0;
			ITree leaf1 = leaves.get(i);
			ITree leaf2 = leaves.get(i+1);
			if(leaf1.getLabel()==""&&leaf2.getLabel()=="")
				continue;
			if(leaf2.getLabel().equals("")&&
					!srcT.getTypeLabel(leaf2).equals("argument_list")&&
					!srcT.getTypeLabel(leaf2).equals("parameter_list")) {
				i++;
				if(i<leaves.size()-1) {
					leaf2 = leaves.get(i+1);
				}else {
					continue;
				}
			}//发现有截取block后的断点影响还原,跳过
			if(leaf1.isRoot()||leaf2.isRoot())//叶子节点为总树根节点，可能么？
				throw new Exception("why is root???");
			ITree sharePar = Utils.findShareParent(leaf1, leaf2, srcT);
//			String parType = srcT.getTypeLabel(sharePar);
			List<ITree> childs = sharePar.getChildren();
			if(childs.contains(leaf1)&&childs.contains(leaf2)) {//同一层的两个叶子节点，还原时候直接拼起来就行
				src = src+" "+leaf2.getLabel();
			}else if(childs.size()>=2){//分情况讨论不同分支下还原代码问题
				ITree node1 = null;
				ITree node2 = null;
				for(ITree child : childs) {
					if(child.isLeaf()) {
						if(child.equals(leaf1))
							node1 = child;
						if(child.equals(leaf2))
							node2 = child;
					}else {
						List<ITree> list = TreeUtils.preOrder(child);
						if(list.contains(leaf1))
							node1 = child;
						if(list.contains(leaf2))
							node2 = child;
//						if(list.contains(leaf1)&&list.contains(leaf1))
//							throw new Exception("wrong sharePar!");
					}
				}//找sharePar的下一个leaf1,leaf2对应父节点(或其本身)
				String type1 = "";
				String type2 = "";
				if(node1!=null&&node2!=null) {
					type1 = srcT.getTypeLabel(node1);
					type2 = srcT.getTypeLabel(node2);
				}else
					System.out.println("why node is null?"+st.getMiName()+","+st.getRoot().getId());

				if(type1.equals("name")) {
					if(type2.equals("argument_list")||type2.equals("parameter_list")) {
						List<ITree> arguLeaves = new ArrayList<>();
						arguLeaves = Utils.traverse2Leaf(node2, arguLeaves);//找到argulist中所有叶子
						src = src + recoverArguList(node2, arguLeaves, srcT);//argulist单独处理
						if(src.substring(src.length()-1)==" ")
							src = src.substring(0, src.length()-1);//去除空格
						size = arguLeaves.size();
						i=i+size-1;
					}else if(type2.equals("init")) {
						src = src+" = "+leaf2.getLabel();
					}else if(type2.equals("operator")) {
						src = src+" "+leaf2.getLabel();
					}else if(type2.equals("modifier")) {
						src = src+" * ";
					}else if(type2.equals("index")) {
						src = src+" [ "+leaf2.getLabel()+" ] ";
					}else {
						src = String.valueOf(st.getRoot().getId())+"error name situation";
						break;
//						throw new Exception("没考虑过的name情况:"+type2);
					}
				}else if(type1.equals("type")) {
					if(type2.equals("name")) {
						src = src+" "+leaf2.getLabel();
					}else {
						src = String.valueOf(st.getRoot().getId())+"error type situation";
						break;
//						throw new Exception("没考虑过的type情况:"+type2);
					}
				}else if(type1.equals("operator")) {
					if(type2.equals("call")) {//好像有node2为call的情况
						node2 = node2.getChildren().get(0);
						type2 = srcT.getTypeLabel(node2);
					}
					if(type2.equals("name")||type2.equals("operator")) {
						src = src+" "+leaf2.getLabel();
					}else {
						src = String.valueOf(st.getRoot().getId())+"error operator situation";
						break;
//						throw new Exception("没考虑过的operator情况:"+type2);
					}
				}else if(type1.equals("call")) {
					if(type2.equals("operator")) {
						src = src+" "+leaf2.getLabel();
					}else if(type2.equals("call")) {
						src = src+" , "+leaf2.getLabel();
					}else {
						src = String.valueOf(st.getRoot().getId())+ "error call situation";
						break;
//						throw new Exception("没考虑过的type情况:"+type2);
					}
				}else if(type1.equals("specifier")) {
					if(type2.equals("name")){
						src = src+" "+leaf2.getLabel();
					}else {
						src = String.valueOf(st.getRoot().getId())+"error specifier situation";
						break;
//						throw new Exception("没考虑过的type情况:"+type2);
					}
				}else if(type1.equals("parameter_list")) {
					if(type2.equals("member_init_list")) {
						src = src+" : "+leaf2.getLabel();
					}
				}else if(type1.equals("decl")) {
					if(type2.equals("decl")) {
						src = src+" , "+leaf2.getLabel();
					}
				}else if(type1.equals("init")) {
					if(type2.equals("condition")) {
						src = src+" ; "+leaf2.getLabel();
					}
				}else if(type1.equals("condition")) {
					if(type2.equals("incr")) {
						src = src+" ; "+leaf2.getLabel();
					}
				}else {
					src = src+"error other situation";
					break;
//					throw new Exception("没考虑过的children情况");
				}
			}
		}
		src = src+loopEnd;
		if(src.contains("  "))
			src = src.replace("  ", " ");
		src = src.trim();
		return src;
	}

	public static String recoverArguList(ITree root, List<ITree> arguLeaves, TreeContext srcT) throws Exception {
		String arguSrc = "";
		String end = "";
		ITree node = root.getParent();
		String type = srcT.getTypeLabel(node);//找到argument_list父节点
		if(type.equals("name")) {//name情况用<>
			arguSrc = arguSrc+" < ";
			end = " > ";
		}else if(type.equals("call")||type.equals("decl")) {//call的情况下用()
			arguSrc = arguSrc+" ( ";
			end = " ) ";
		}else if(type.equals("constructor")||type.equals("function")) {
			arguSrc = arguSrc+" ( ";
			end = " ) ";
		}
		if(arguLeaves.size()==0) {
			arguSrc = arguSrc+end;
			return 	arguSrc;
		}//返回空括号
		if(arguLeaves.size()==1) {
			arguSrc = arguSrc + deleteLiteral(arguLeaves.get(0), srcT)+end;
			return 	arguSrc;
		}//返回单个元素+括号

		arguSrc = arguSrc + deleteLiteral(arguLeaves.get(0), srcT);
		for(int i=0;i<arguLeaves.size()-1;i++) {
			ITree leaf1 = arguLeaves.get(i);
			ITree leaf2 = arguLeaves.get(i+1);
			ITree sharePar = Utils.findShareParent(leaf1, leaf2, srcT);
//			String parType = srcT.getTypeLabel(sharePar);
			List<ITree> childs = sharePar.getChildren();
			if(childs.contains(leaf1)&&childs.contains(leaf2)) {//同一层的两个叶子节点，还原时候直接拼起来就行
				arguSrc = arguSrc+" "+deleteLiteral(leaf2, srcT);
			}else if(childs.size()>=2){
				ITree node1 = null;
				ITree node2 = null;
				for(ITree child : childs) {
					if(child.isLeaf()) {
						if(child.equals(leaf1))
							node1 = child;
						if(child.equals(leaf2))
							node2 = child;
					}else {
						List<ITree> list = TreeUtils.preOrder(child);
						if(list.contains(leaf1))
							node1 = child;
						if(list.contains(leaf2))
							node2 = child;
//						if(list.contains(leaf1)&&list.contains(leaf1))
//							throw new Exception("wrong sharePar!");
					}
				}//找sharePar的下一个leaf1,leaf2对应父节点(或其本身)
				String type1 = srcT.getTypeLabel(node1);
				String type2 = srcT.getTypeLabel(node2);
				if(type1.equals("name")) {
					if(type2.equals("argument_list")||type2.equals("parameter_list")) {
						List<ITree> leaves = new ArrayList<>();
						leaves = Utils.traverse2Leaf(node2, leaves);//找到argulist中所有叶子
						arguSrc = arguSrc + recoverArguList(node2, leaves, srcT);
					}else if(type2.equals("operator")) {
						arguSrc = arguSrc+" "+deleteLiteral(leaf2, srcT);
					}else if(type2.equals("modifier")) {
						arguSrc = arguSrc+" * ";
					}else {
						arguSrc = String.valueOf(srcT.getRoot().getId())+"error nameArg situation";
						break;
//						throw new Exception("没考虑过的name情况:"+type2);
					}
				}else if(type1.equals("argument")||type1.equals("parameter")) {
					if(type2.equals("argument")||type2.equals("parameter")) {
						arguSrc = arguSrc+" , "+deleteLiteral(leaf2, srcT);
					}else {
						arguSrc = String.valueOf(srcT.getRoot().getId())+"error argumentArg situation";
						break;
//						throw new Exception("没考虑过的argument情况:"+type2);
					}
				}else if(type1.equals("type")) {
					if(type2.equals("name")) {
						arguSrc = arguSrc+" "+deleteLiteral(leaf2, srcT);
					}else {
						arguSrc = String.valueOf(srcT.getRoot().getId())+"error typeArg situation";
						break;
//						throw new Exception("没考虑过的type情况:"+type2);
					}
				}else if(type1.equals("call")) {
					if(type2.equals("operator")) {
						arguSrc = arguSrc+" "+deleteLiteral(leaf2, srcT);
					}else {
						arguSrc = String.valueOf(srcT.getRoot().getId())+"error callArg situation";
						break;
//						throw new Exception("没考虑过的type情况:"+type2);
					}
				}else if(type1.equals("operator")) {
					if(type2.equals("call")) {//好像有node2为call的情况
						node2 = node2.getChildren().get(0);
						type2 = srcT.getTypeLabel(node2);
					}
					if(type2.equals("name")||type2.equals("operator")) {
						arguSrc = arguSrc+" "+deleteLiteral(leaf2, srcT);
					}else {
						arguSrc = String.valueOf(srcT.getRoot().getId())+"error operatorArg situation";
						break;
//						throw new Exception("没考虑过的operator情况:"+type2);
					}
				}else if(type1.equals("specifier")) {
					if(type2.equals("name")){
						arguSrc = arguSrc+" "+deleteLiteral(leaf2, srcT);
					}else {
						arguSrc = String.valueOf(srcT.getRoot().getId())+"error specifierArg situation";
						break;
//						throw new Exception("没考虑过的type情况:"+type2);
					}
				}else {
					arguSrc = String.valueOf(srcT.getRoot().getId())+"error otherArg situation";
					break;
//					throw new Exception("没考虑过的children情况");
				}
			}
		}
		arguSrc = arguSrc+end;//加上收尾
		return arguSrc;
	}//argulist相当于subtree中的subtree，单独还原

	public static String deleteLiteral(ITree leaf, TreeContext tc) {
		String label = leaf.getLabel();
		String type = tc.getTypeLabel(leaf);
		if(type.equals("literal")) {
//			if(isNumber(label)==true) {
//				if(isInteger(label)==true) {
//					label = "Int";
//				}else if(isDouble(label)==true){
//					label = "Float";
//				}
//			}
			if(label.contains("\""))
				label = "\"\"";
		}
		return label;
	}

	public static String absVariable(String src, HashMap<String, String> varMap) {
		String[] srcs = src.split(" ");
		String newLine = "";
		if(srcs.length==1) {
			String token = srcs[0];
			if(varMap.containsKey(token)) {
				if(newLine.equals("")) {
					newLine = varMap.get(token);
				}else
					newLine = newLine+" "+varMap.get(token);
			}else {
				if(newLine.equals("")) {
					newLine = token;
				}else
					newLine = newLine+" "+token;
			}
		}else {
			for(int i=0;i<srcs.length-1;i++) {
				String token = srcs[i];
				String nextToken = srcs[i+1];
				if(varMap.containsKey(token)) {
					if(newLine.equals("")) {
						newLine = varMap.get(token);
					}else {
						if(nextToken.equals("(")) {
							newLine = newLine+" "+token;
						}else
							newLine = newLine+" "+varMap.get(token);
					}
				}else {
					if(newLine.equals("")) {
						newLine = token;
					}else
						newLine = newLine+" "+token;
				}
			}
			newLine = newLine+" "+srcs[srcs.length-1];//补上尾巴
		}
		return newLine;
	}

	private static boolean isNumber (Object obj) {
		if (obj instanceof Number) {
			return true;
		} else if (obj instanceof String){
			try{
				Double.parseDouble((String)obj);
				return true;
			}catch (Exception e) {
				return false;
			}
		}
		return false;
	}

	/*
	  * 判断是否为整数
	  * @param str 传入的字符串
	  * @return 是整数返回true,否则返回false
	*/
	  public static boolean isInteger(String str) {
	    Pattern pattern = Pattern.compile("^[-\\+]?[\\d]*$");
	    return pattern.matcher(str).matches();
	  }


	/*
	  * 判断是否为浮点数，包括double和float
	  * @param str 传入的字符串
	  * @return 是浮点数返回true,否则返回false
	*/
	  public static boolean isDouble(String str) {
	    Pattern pattern = Pattern.compile("^[-\\+]?[.\\d]*$");
	    return pattern.matcher(str).matches();
	  }
}
