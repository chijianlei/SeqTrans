package utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import gumtreediff.actions.model.Action;
import gumtreediff.io.TreeIoUtils;
import gumtreediff.matchers.MappingStore;
import gumtreediff.tree.ITree;
import gumtreediff.tree.Tree;
import gumtreediff.tree.TreeContext;
import split.Split;
import structure.Definition;
import structure.Migration;
import structure.SubTree;

public class Defuse {
	public ArrayList<ITree> blocks1 = new ArrayList<>();
	public ArrayList<ITree> blocks2 = new ArrayList<>();
	public HashMap<ITree, ArrayList<ITree>> parBlockMap1 = new HashMap<>();
	public HashMap<ITree, ArrayList<ITree>> parBlockMap2 = new HashMap<>();
	public ArrayList<Definition> globalDefs1 = new ArrayList<>();
	public ArrayList<Definition> globalDefs2 = new ArrayList<>();
	public ArrayList<Definition> defs1 = new ArrayList<>();
	public ArrayList<Definition> defs2 = new ArrayList<>();

	public static void main (String args[]) throws Exception{
		String path = "migrations_test1";
		Defuse defuse = new Defuse();
		defuse.collectDiffwithDefUse(path, "txt", true, false, "");
	}

	public void collectDiffwithDefUse(String path, String outMode,
		Boolean ifOnlyChange, Boolean ifPrintDef, String filter) throws Exception {//获取DefUse
		Split sp = new Split();
		ArrayList<Migration> migrats = sp.readMigration(path, filter);
		String jpath = "jsons\\";
		File jFile = new File(jpath);
		if(!jFile.exists())
			jFile.mkdirs();
		int count = 0;//计数
		if(jFile.listFiles().length!=0) {
			throw new Exception("Plz clean dir!");
		}
		File output = new File("defuse.txt");
		BufferedWriter wr = new BufferedWriter(new FileWriter(output));
		File output1 = new File("src-val.txt");
		File output2 = new File("tgt-val.txt");
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(output1));
		BufferedWriter wr2 = new BufferedWriter(new FileWriter(output2));
//		ArrayList<String> includes1 = readIncludes("src");
//		ArrayList<String> includes2 = readIncludes("dst");
//		for(String include : includes1) {
//			wr1.append(include);
//			wr1.newLine();
//			wr1.flush();
//		}
//		for(String include : includes2) {
//			wr2.append(include);
//			wr2.newLine();
//			wr2.flush();
//		}

		int errCount = 0;
		String tmpName = "";
		for(Migration migrat : migrats) {
			blocks1.clear();
			blocks2.clear();
			parBlockMap1.clear();
			parBlockMap2.clear();
			String miName = migrat.getMiName();
			TreeContext sTC = migrat.getSrcT();
			TreeContext dTC = migrat.getDstT();
			MappingStore mappings = migrat.getMappings();

			System.out.println("Analyse:"+miName);
			ArrayList<SubTree> changedSTree = new ArrayList<>();
			HashMap<String, LinkedList<Action>> actions = Utils.collectAction(sTC, dTC, mappings);
			ArrayList<Integer> srcActIds = Utils.collectSrcActNodeIds(sTC, dTC, mappings, actions);
			ArrayList<Definition> defs1 = getDef(sTC, "src");//先计算action,再收集defs
	        ArrayList<Definition> defs2 = getDef(dTC, "tgt");
	        HashMap<String, ArrayList<Definition>> defMap1 = transferDefs(defs1);
	        HashMap<String, ArrayList<Definition>> defMap2 = transferDefs(defs2);
	        HashMap<ITree, ArrayList<Definition>> blockMap1 = transferBlockMap(defs1, sTC, "src");
	        HashMap<ITree, ArrayList<Definition>> blockMap2 = transferBlockMap(defs2, dTC, "tgt");
			ArrayList<SubTree> sub1 = sp.splitSubTree(sTC, miName);//Subtree中割裂过block,注意
			ArrayList<SubTree> sub2 = sp.splitSubTree(dTC, miName);//先计算action,再split ST

			System.out.println(defs1.size());
	        System.out.println(defs2.size());
	        System.out.println(defMap1.size());
	        System.out.println(defMap2.size());
	        System.out.println(blockMap1.size());
	        System.out.println(blockMap2.size());

	        for(SubTree st : sub1) {
	        	ITree root = st.getRoot();
	        	System.out.println("StID:"+root.getId());
	        }
	        if(ifOnlyChange) {
				for(SubTree st : sub1) {
					ITree t = st.getRoot();
					List<ITree> nodeList = t.getDescendants();
					nodeList.add(t);
		        	for(ITree node : nodeList) {
		        		int id = node.getId();
		        		if(srcActIds.contains(id)) {
		        			changedSTree.add(st);
//		        			System.out.println("find a action subtree!");
		        			break;
		        		}
		        	}
				}//先找包含action的subtree
	        }else {
	        	changedSTree = sub1;
	        }

	        System.out.println("subSize:"+sub1.size());
			System.out.println("changeSize:"+changedSTree.size());
			for(SubTree srcT : changedSTree) {
				System.out.println("===================");
				HashSet<Definition> usedDefs1 = new HashSet<>();
				HashSet<Definition> usedDefs2 = new HashSet<>();
				ITree sRoot = srcT.getRoot();
				System.out.println("CheckMapping "+sRoot.getId()+":"+srcT.getMiName());
				String src = Output.subtree2src(srcT);


	    		if(outMode=="txt") {
	    			if(src.contains("error")&&src.contains("situation")) {
	    				errCount++;
	    				continue;
	    			}
	    		}

	    		Boolean same = false;
				ArrayList<ITree> leaves1 = new ArrayList<>();
				Utils.traverse2Leaf(sRoot, leaves1);
				int labelCount = 0;
				for(ITree leaf : leaves1) {
					String label = leaf.getLabel();
					System.out.println("label"+label);
					if(!label.equals(""))
						labelCount++;
					String type = sTC.getTypeLabel(leaf);
					if(type.equals("literal")) {
						leaf.setLabel(Output.deleteLiteral(leaf, sTC));
					}
					ArrayList<Definition> stringList = defMap1.get(label);
					if(stringList!=null) {
						ITree parBlock = searchBlock(leaf, sTC);
						ArrayList<Definition> blockList = blockMap1.get(parBlock);
						for(Definition def1 : stringList) {
							if(blockList!=null) {
								if(blockList.contains(def1)) {
									if(leaf.getId()>def1.getDefLabelID()) {
										leaf.setLabel("var");
										usedDefs1.add(def1);
									}
								}
							}
							if(def1.getDefLabelID()==leaf.getId()) {
								leaf.setLabel("var");
							}
							System.out.println(leaf.getId()+","+def1.getDefLabelID());
							System.out.println(def1.getType()+","+def1.getVarName());
						}
					}
				}
				if(labelCount<=1)
					continue;

				SubTree dstT = checkMapping(srcT, mappings, dTC, sub2);
				if(dstT==null)
					continue;//子树没有对应子树，被删除
				ITree dRoot = dstT.getRoot();
	    		System.out.println(sRoot.getId()+"->"+dRoot.getId());

				ArrayList<ITree> leaves2 = new ArrayList<>();
				Utils.traverse2Leaf(dRoot, leaves2);
				for(ITree leaf : leaves2) {
					String label = leaf.getLabel();
					String type = dTC.getTypeLabel(leaf);
					if(type.equals("literal")) {
						leaf.setLabel(Output.deleteLiteral(leaf, dTC));
					}
					ArrayList<Definition> stringList = defMap2.get(label);
					if(stringList!=null) {
						ITree parBlock = searchBlock(leaf, dTC);
						ArrayList<Definition> blockList = blockMap2.get(parBlock);
						for(Definition def2 : stringList) {
							if(blockList!=null) {
								if(blockList.contains(def2)) {
									if(leaf.getId()>def2.getDefLabelID()) {
										usedDefs2.add(def2);
										leaf.setLabel("var");
									}
									System.out.println(leaf.getId()+","+def2.getDefLabelID());
									System.out.println(def2.getType()+","+def2.getVarName());
								}
							}
							if(def2.getDefLabelID()==leaf.getId()) {
								leaf.setLabel("var");
							}
						}
					}
					if(!same) {
						for(ITree leaf1 : leaves1) {
							String label1 = leaf1.getLabel();
							if(label.equals(label1)) {
								same = true;
							}
						}
					}
				}
				src = Output.subtree2src(srcT);
	    		String tar = Output.subtree2src(dstT);
	    		if(outMode=="txt") {
		    		if(tar.contains("error")&&tar.contains("situation")) {
		    			errCount++;
		    			continue;
		    		}
	    		}
	    		if(ifOnlyChange) {
	    			if(src.equals(tar))
		    			continue;
	    		}

	    		//长度相差太多的句子直接跳过
				if(((float)src.length()/(float)tar.length())<0.25||((float)tar.length()/(float)src.length())<0.25 || !same)
					continue;//no leaf is the same
				if(outMode=="txt") {
					if(ifPrintDef) {
						String buffer = "";
						String buffer1 = "";
						String buffer2 = "";
						for(Definition def : usedDefs1) {
							SubTree st = new SubTree(def.getRoot(), sTC, 0, "");
							String stat = Output.subtree2src(st);
							buffer = buffer +stat+" ; ";
							buffer1 = buffer1 +stat+" ; ";
						}
						src = Output.subtree2src(srcT);
//						src += String.valueOf(srcT.getRoot().getId());
						buffer = buffer + src+"\t";
						buffer1 += src;
						for(Definition def : usedDefs2) {
							SubTree st = new SubTree(def.getRoot(), dTC, 0, "");
							String stat = Output.subtree2src(st);
							buffer += stat+" ; ";
							buffer2 += stat+" ; ";
						}
						tar = Output.subtree2src(dstT);
//						tar += String.valueOf(dstT.getRoot().getId());
						buffer += tar;
						buffer2 += tar;
						if(buffer.contains("error")&&buffer.contains("situation"))
							continue;
						wr.append(buffer);
						wr1.append(buffer1);
						wr2.append(buffer2);
						wr1.newLine();
						wr1.flush();
						wr2.newLine();
						wr2.flush();
						wr.newLine();
						wr.flush();
					}else {
						src += String.valueOf(srcT.getRoot().getId());
						tar += String.valueOf(dstT.getRoot().getId());
			    		String buffer = src+"\t"+tar;
						if(buffer.contains("error")&&buffer.contains("situation"))
							continue;
						wr.append(buffer);
						wr1.append(src);
						wr2.append(tar);
						wr1.newLine();
						wr1.flush();
						wr2.newLine();
						wr2.flush();
						wr.newLine();
						wr.flush();
					}
				}else if(outMode=="json") {
					TreeContext st = buildTC(srcT);
					TreeContext dt = buildTC(dstT);
					printJson(jpath, count, st, dt);
					count++;
				}
			}
		}
		wr.close();
		wr1.close();
		wr2.close();
		System.out.println("miname:"+tmpName);
		System.out.println("errCount:"+errCount);
	}


	static private ArrayList<ArrayList<SubTree>> searchQueue(Queue<SubTree> qe, ArrayList<SubTree> changedSTree, HashMap<String, ArrayList<Integer>> sDefMap) throws Exception {
		ArrayList<ArrayList<SubTree>> resultList = new ArrayList<>();
		HashMap<Integer, ArrayList<SubTree>> resultMap = new HashMap<>();
		for(SubTree st : qe) {
			TreeContext tc = st.getTC();
			if(!changedSTree.contains(st))
				continue;//只考虑需要修改的语句
			ITree sRoot = st.getRoot();
			List<ITree> sLeaves = new ArrayList<>();
			Utils.traverse2Leaf(sRoot, sLeaves);
			for(ITree leaf : sLeaves) {
				String type = tc.getTypeLabel(leaf);
				if(type.equals("name")) {
					String label = leaf.getLabel();
					ArrayList<Integer> map = sDefMap.get(label);
					if(map!=null) {
						if(map.size()==1) {
							int defID = map.get(0);
							if(resultMap.get(defID)==null) {
								ArrayList<SubTree> stList = new ArrayList<>();
								stList.add(st);
								resultMap.put(defID, stList);
							}else {
								resultMap.get(defID).add(st);
							}
						}else {//发现有多个def line同一个关键字的情况，可能发生在不同的method
							Collections.sort(map);
							ArrayList<Integer> subMap = new ArrayList<>();
							for(int id : map) {//只取该条语句之前的subtree,抛弃掉之后的
								if(id<sRoot.getId())
									subMap.add(id);
							}
							if(subMap.size()==0)
								continue;//好像有defid全在rootid之后的情况，跳过
							int defID = subMap.get(subMap.size()-1);//取离该def最近的
							if(resultMap.get(defID)==null) {
								ArrayList<SubTree> stList = new ArrayList<>();
								stList.add(st);
								resultMap.put(defID, stList);
							}else {
								resultMap.get(defID).add(st);
							}
							System.out.println("mLine");
						}
					}
				}
			}
		}
		for(Map.Entry<Integer, ArrayList<SubTree>> entry : resultMap.entrySet()) {
			ArrayList<SubTree> list = entry.getValue();
			if(list.size()>=2) {
				resultList.add(list);
			}
		}
		return resultList;
	}//搜索队列，找到有data dependency的语句

	private static Definition searchPramDef(ArrayList<Definition> defs, ITree node) {
		for(Definition def : defs) {
			ITree root = def.getRoot();
			if(root.equals(node))
				return def;
		}
		return null;
	}

	public HashMap<ITree, ITree> searchBlockMap(TreeContext tc) throws Exception{
		HashMap<ITree, ITree> leaf2parblock_map = new HashMap<>();
		ITree root = tc.getRoot();
		List<ITree> leaves = new ArrayList<>();
		leaves = Utils.traverse2Leaf(root, leaves);
		for(ITree leaf : leaves) {
			ITree parBlock = searchBlock(leaf, tc);
			if(parBlock!=null) {
				leaf2parblock_map.put(leaf, parBlock);
			}
		}
		return leaf2parblock_map;
	}

	public ITree searchBlock(ITree node, TreeContext tc) {
		List<ITree> pars = node.getParents();
		for(ITree par : pars) {
			if(tc.getTypeLabel(par).equals("block")) {
				return par;
			}
		}
		return null;
	}

	private static ArrayList<ITree> searchNextBlocks(Definition def, TreeContext tc) throws Exception {
		ITree defRoot = def.getRoot();
		List<ITree> pars = defRoot.getParents();
		ITree func = null;
		if(!isParameter(def, tc))
			throw new Exception("Only used for parameters");
//		System.out.println("--------------");
		for(ITree par : pars) {
			String type = tc.getTypeLabel(par);
//			System.out.println(type);
			if(type.equals("function")||type.equals("constructor")||type.equals("function_decl")||
					type.equals("catch")||type.equals("lambda")) {
				func = par;
				break;
			}
		}
		ArrayList<ITree> childBlocks = new ArrayList<>();
		if(func==null)
//			throw new Exception("Error def"+defRoot.getId()+tc.getTypeLabel(defRoot));
			return childBlocks;
		List<ITree> des = func.getDescendants();
		for(ITree node : des) {
			String type = tc.getTypeLabel(node);
			if(type.equals("block"))
				childBlocks.add(node);
		}
		return childBlocks;
	}

	public SubTree checkMapping(SubTree st, MappingStore map, TreeContext tc2, List<SubTree> sub2) throws Exception {
		ITree root = st.getRoot();
		int sSize = st.getTC().getSize();
		int dSize = tc2.getSize();
		ITree dstRoot = map.getDst(root);
		List<ITree> desList = root.getDescendants();
		ArrayList<ITree> seeds = new ArrayList<>();
		Boolean sameLeaf = false;
		HashMap<ITree, Integer> dRootCandidates = new HashMap<>();
		int max_frequency = 0;
		int print_seedID = 0;
		for(ITree node : desList) {
			ITree dst = map.getDst(node);
			if(dst!=null) {
				if(node.isLeaf()&&dst.isLeaf()) {
					if(legalLebelMap(node, sSize, dst, tc2.getSize())) {
						sameLeaf = true;
					}
				}
				seeds.add(dst);
			}
		}
//		System.out.println("SameLeaf:"+sameLeaf);
//		System.out.println("Seeds:"+seeds.size());
		if(!sameLeaf || (seeds.size()==0))
			return null;
//		System.out.println("sRootID:"+root.getId());
		if(dstRoot==null) {
			for(ITree seed : seeds) {
				print_seedID = seed.getId();
				List<ITree> pars = seed.getParents();
//				System.out.println(pars.size());
				for(ITree par : pars) {
//					System.out.println(seed.getId()+","+tc2.getTypeLabel(par));
					if(Utils.ifSRoot(tc2.getTypeLabel(par))) {
						Integer frequency = dRootCandidates.get(par);
						if(frequency==null) {
							dRootCandidates.put(par, 1);
						}else {
							frequency++;
							dRootCandidates.put(par, frequency);
						}
						break;
					}
				}
			}

			for(Map.Entry<ITree, Integer> entry : dRootCandidates.entrySet()) {
				ITree candi = entry.getKey();
				int frequency = entry.getValue();
				if(frequency>max_frequency) {
					dstRoot = candi;
					max_frequency = frequency;
				}else if(frequency==max_frequency) {
					float sLocation = (float)root.getId()/(float)sSize;
					float dLocation_old = (float)dstRoot.getId()/(float)dSize;
					float dLocation_new = (float)candi.getId()/(float)dSize;
					if(Math.abs(dLocation_new-sLocation)<Math.abs(dLocation_old)-sLocation) {
						dstRoot = candi;
						max_frequency = frequency;
					}//如果frequcency相等，取location位置与sRoot更接近的dRoot
				}
			}//发现在subtree中有dst node分布在多棵subtree中的情况

			if(dstRoot==null) {
				System.err.println("error dstRoot:"+print_seedID);
				return null;
			}
			List<ITree> desList2 = dstRoot.getDescendants();
			for(ITree node : seeds) {
				if(!desList2.contains(node)) {
					return null;
				}
			}//是否允许两个子树中分布有不在这两颗子树内的mapping?
		}

		for(SubTree dstT : sub2) {
			ITree candi = dstT.getRoot();
			if(candi.equals(dstRoot))
				return dstT;
		}
		return null;
	}//检查与srcST符合的dstST mapping

	private static Boolean legalLebelMap(ITree src, int sSize, ITree dst, int dSize) {
		String label1 = src.getLabel();
		String label2 = dst.getLabel();
		boolean isSame = false;
		if(label1==null||label2==null) {
			System.err.println("error label:"+src.getId());
			return isSame;
		}
		float sLocation = (float)src.getId()/(float)sSize;
		float dLocation = (float)dst.getId()/(float)dSize;
		if(Math.abs(sLocation-dLocation)>=0.33)
			return isSame;//如果两个label在两份code中位置相差过远，不考虑相同
		if(label1.equals(label2)&&!label1.equals("::")&&!label2.equals("::"))
			isSame = true;
		if(label1.equals("ros")&&label2.equals("rclcpp"))
			isSame = true;
		if(label1.equals("tf")&&label2.equals("tf2"))
			isSame = true;
		return isSame;
	}//label是否相同的相关规则

	public static TreeContext abstraSubTree(SubTree st, HashMap<String, String> varMap) {
		TreeContext origin = st.getTC();
		TreeContext subT = new TreeContext();
		subT.importTypeLabels(origin);
		ITree subRoot = st.getRoot();
		subT.setRoot(subRoot);
		List<ITree> descendants = subRoot.getDescendants();
		for(ITree node : descendants) {
			String label = node.getLabel();
			String type = subT.getTypeLabel(node);
			if (varMap.containsKey(label)) {
				System.out.println("find it!");
				node.setLabel("var");
			}
			if(type.equals("literal")) {
				if (label.contains("\"")) {
					label = "stringliteral";
				}else {
					label = "intliteral";
				}
				node.setLabel(label);
			}
		}
		return subT;
	}

	static private TreeContext abstraTotalTC(ArrayList<SubTree> stList, HashMap<String, String> varMap) {
		TreeContext origin = stList.get(0).getTC();
		TreeContext subT = new TreeContext();
		subT.importTypeLabels(origin);
		subT.registerTypeLabel(000, "Block");
		Tree blockRoot = new gumtreediff.tree.Tree(000, null);
		List<ITree> children = new ArrayList<>();
		for(SubTree st : stList) {
			children.add(st.getRoot());
		}
		blockRoot.setChildren(children);
		subT.setRoot(blockRoot);
		List<ITree> descendants = blockRoot.getDescendants();
		for(ITree node : descendants) {
			String label = node.getLabel();
			String type = subT.getTypeLabel(node);
			if (varMap.containsKey(label)) {
				System.out.println("find it!");
				node.setLabel("var");
			}
			if(type.equals("literal")) {
				if (label.contains("\"")) {
					label = "stringliteral";
				}else {
					label = "intliteral";
				}
				node.setLabel(label);
			}
		}
		//创建block节点，输出拼起来的总图，然后把剩下的单行输出
		return subT;
	}

	public TreeContext buildTC(SubTree st) {
		TreeContext origin = st.getTC();
		TreeContext subT = new TreeContext();
		ITree root = st.getRoot();
		subT.importTypeLabels(origin);
		subT.setRoot(root);
		//创建block节点，输出拼起来的总图，然后把剩下的单行输出
		return subT;
	}

	public void printJson(String jpath, int count, TreeContext srcT, TreeContext dstT) throws Exception {
		File dir = new File(jpath);
		if(!dir.exists()) {
			dir.mkdirs();
		}
		if(srcT!=null) {
			String out = jpath+"pair"+String.valueOf(count)+"_src.json";
			BufferedWriter wr = new BufferedWriter(new FileWriter(new File(out)));
			wr.append(TreeIoUtils.toJson(srcT).toString());
			wr.flush();
			wr.close();
		}
		if(dstT!=null) {
			String out1 = jpath+"pair"+String.valueOf(count)+"_tgt.json";
			BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File(out1)));
			wr1.append(TreeIoUtils.toJson(dstT).toString());
			wr1.flush();
			wr1.close();
		}
	}

	public ArrayList<Definition> getDef(TreeContext tc, String from) throws Exception {
		ArrayList<Definition> defs = new ArrayList<>();
		HashMap<ITree, ArrayList<ITree>> parBlockMap = new HashMap<>();
		ArrayList<ITree> blocks = new ArrayList<>();
		ITree root = tc.getRoot();
		List<ITree> allNodes = root.getDescendants();
		for(ITree node : allNodes) {
			if(tc.getTypeLabel(node).equals("block")) {
				List<ITree> pars = node.getParents();
				ArrayList<ITree> parBlocks = new ArrayList<>();
				for(ITree par : pars) {//search block parents
					String type = tc.getTypeLabel(par);
					if(type.equals("blcok")) {
						parBlocks.add(par);
					}
				}
				parBlockMap.put(node, parBlocks);
				blocks.add(node);
			}else if(tc.getTypeLabel(node).equals("decl")) {
//				System.out.println("find def: "+node.getId());
				Definition def = new Definition();
				def.setRoot(node);
				Boolean hasName = false;
				List<ITree> children = node.getChildren();
				for(ITree child : children) {
					String type = tc.getTypeLabel(child);
					if(type.equals("type")) {
						String typeName = "";
						List<ITree> typeChilds = child.getChildren();
						for(ITree child1 : typeChilds) {
							if(child1.isLeaf()) {//单label情况
								String label = child1.getLabel();
								typeName = typeName+label;
								if(label==null)
									throw new Exception("check ITree: "+child1.getId());
							}else {
								for(ITree leaf : child1.getChildren()) {
									String label = leaf.getLabel();
									typeName = typeName+label;
								}
							}
						}
						def.setType(typeName);
					}else if(type.equals("name")) {
						String label = child.getLabel();
						if(label==null)
							throw new Exception("check ITree: "+child.getId());
						def.setVarName(label);//varName
						def.setDefLabelID(child.getId());
						hasName = true;
					}
				}
				if(!hasName) {
					continue;
				}
				List<ITree> pars = node.getParents();
				List<ITree> parBlocks = new ArrayList<>();
				List<ITree> parList = new ArrayList<>();
				Boolean first = true;
				for(ITree par : pars) {//search block parents
					String type = tc.getTypeLabel(par);
					if(type.equals("block")) {
						if(first) {
							def.setBlock(par);
							first = false;
						}
						parBlocks.add(par);
					}
					parList.add(par);
				}
				if(parBlocks.size()==0) {
					parBlocks.add(tc.getRoot());
					def.setBlock(tc.getRoot());
				}//全局节点没有父亲block，直接默认block为tc根节点

				def.setParBlocks(parBlocks);
				def.setParList(parList);
				def.setTc(tc);
				defs.add(def);
			}
		}
		
		ArrayList<ITree> global_decls = getGlobalDefRoots(tc);
		ArrayList<Definition> globalDefs = new ArrayList<>();
		for(Definition def : defs) {
			ITree defRoot = def.getRoot();
			if(global_decls.contains(defRoot))
				globalDefs.add(def);
		}
		
		if(from.equals("src")) {
			blocks1 = blocks;
			parBlockMap1 = parBlockMap;
			defs1 = defs;
			globalDefs1 = globalDefs;
		}else if(from.equals("tgt")) {
			blocks2 = blocks;
			parBlockMap2 = parBlockMap;
			defs2 = defs;
			globalDefs2 = globalDefs;
		}
		return defs;
	}
	
	public ArrayList<ITree> getGlobalDefRoots(TreeContext tc){
		ArrayList<ITree> global_decls = new ArrayList<>();
		ITree root = tc.getRoot();
		List<ITree> allNodes = root.getDescendants();
		List<ITree> classNodes = new ArrayList<>();
		for(ITree node : allNodes) {
			if(tc.getTypeLabel(node).equals("class")) {
				classNodes.add(node);
			}
		}
		for(ITree node : classNodes) {
			List<ITree> childs = node.getChildren();
			for(ITree child : childs) {
				if(tc.getTypeLabel(child).equals("block")) {
					ITree blockRoot = child;
					for(ITree child1 : blockRoot.getChildren()) {
						if(tc.getTypeLabel(child1).equals("decl_stmt")) {
							for(ITree child2 : child1.getChildren()) {
								if(tc.getTypeLabel(child1).equals("decl")) {
									global_decls.add(child2);
									break;
								}
							}							
						}
					}
					break;
				}
			}
		}
		return global_decls;
	}

	public HashMap<ITree, ArrayList<Definition>> transferBlockMap(ArrayList<Definition> defs, TreeContext tc, String from) throws Exception {
		HashMap<ITree, ArrayList<Definition>> blockMap = new HashMap<>();
		ArrayList<ITree> blocks = new ArrayList<>();
		if(from.equals("src")) {
			blocks = blocks1;
		}else if(from.equals("tgt")) {
			blocks = blocks2;
		}
		for(ITree block : blocks) {
			ITree parNode = block.getParent();
			String parType = tc.getTypeLabel(parNode);
			List<ITree> pars = block.getParents();
			List<ITree> blockList = new ArrayList<>();
			blockList.add(block);
			for(ITree par : pars) {//search block parents
				String type = tc.getTypeLabel(par);
				if(type.equals("block")) {
					blockList.add(par);
				}
			}
			if(blockList.size()==0)
				throw new Exception("check parBlock: "+block.getId());
			ArrayList<Definition> defList = new ArrayList<>();
			for(Definition def : defs) {
				ITree parBlcok = def.getBlock();
				if(!isParameter(def, tc)) {//所有非参数且block位于目标block的父亲block列表中的def放入list
					if(blockList.contains(parBlcok)) {
						defList.add(def);
					}
				}else {
					ArrayList<ITree> nextBlocks = searchNextBlocks(def, tc);
					if(nextBlocks.contains(block)) {
						defList.add(def);
					}//参数def只在所有子block中生效
				}
				if(parBlcok.equals(tc.getRoot())) {
					if(!isParameter(def, tc)) {
						defList.add(def);
					}
				}//发现有全局变量情况，父亲节点没有root，单独处理，放入所有defList中
			}
			if(parType.equals("function")) {//function parameters
				for(ITree node : parNode.getChildren()) {
					String type = tc.getTypeLabel(node);
					if(type.equals("parameter_list")&&!node.isLeaf()) {
						List<ITree> desendants = node.getDescendants();
						for(ITree child : desendants) {
							if(tc.getTypeLabel(child).equals("decl")) {
								Definition def = searchPramDef(defs, node);
								defList.add(def);
							}
						}
					}
				}
			}
			blockMap.put(block, defList);
		}
		return blockMap;
	}

	private static Boolean isParameter(Definition def, TreeContext tc) {
		boolean isPatameter = false;
		ITree root = def.getRoot();
		ITree par = root.getParent();
		String type = tc.getTypeLabel(par);
		if(type.equals("parameter")) {
			isPatameter = true;
			return isPatameter;
		}else {
			return isPatameter;
		}
	}

	public HashMap<String, ArrayList<Definition>> transferDefs(ArrayList<Definition> defs) {
		HashMap<String, ArrayList<Definition>> defMap = new HashMap<>();
		for(Definition def : defs) {
			String var = def.getVarName();
			if(defMap.get(var)==null) {
				ArrayList<Definition> list = new ArrayList<>();
				list.add(def);
				defMap.put(var, list);
			}else {
				defMap.get(var).add(def);
			}
		}
		return defMap;
	}

	public HashMap<ITree, ArrayList<Definition>> transfer2FunctionMap(ArrayList<Definition> defs, ITree root, TreeContext tc) {
		HashMap<ITree, ArrayList<Definition>> def_functionMap = new HashMap<>();
		for(Definition def : defs) {
		    List<ITree> pars = def.getParList();
		    for(int i=0;i<pars.size();i++) {
		    	ITree par = pars.get(i);
		    	String type = tc.getTypeLabel(par);
		    	if(type.equals("function")) {
		    		if(def_functionMap.get(par)==null) {
		    			ArrayList<Definition> tmps = new ArrayList<>();
		    			tmps.add(def);
		    			def_functionMap.put(par, tmps);
		    		}else
		    			def_functionMap.get(par).add(def);
		    		break;//只在最近的function内生效
		    	}
		    	if(i==pars.size()) {
		    		if(def_functionMap.get(root)==null) {
		    			ArrayList<Definition> tmps = new ArrayList<>();
		    			tmps.add(def);
		    			def_functionMap.put(root, tmps);
		    		}else
		    			def_functionMap.get(root).add(def);
		    	}
		    }
		}
		return def_functionMap;
	}

	private static ArrayList<String> readIncludes(String from) throws Exception {
		ArrayList<String> includes = new ArrayList<>();
		String path = "tmp//includes.txt";
		File file = new File(path);
		BufferedReader br = new BufferedReader(new FileReader(file));
		String tmpline = "";
		while((tmpline=br.readLine())!=null) {
			String include = "";
			if(from=="src") {
				include = tmpline.split("\t")[0];
			}else if(from=="dst") {
				include = tmpline.split("\t")[1];
			}
			includes.add(include);
		}
		br.close();
		return includes;
	}













}
