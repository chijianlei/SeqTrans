package gumtreediff.io;

import java.io.Writer;
import java.util.HashMap;
import java.util.List;

import gumtreediff.actions.model.Action;
import gumtreediff.actions.model.Delete;
import gumtreediff.actions.model.Insert;
import gumtreediff.actions.model.Move;
import gumtreediff.actions.model.Update;
import gumtreediff.io.TreeIoUtils.TreeFormatterAdapter;
import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class DetailedDotFormatter extends TreeFormatterAdapter {
    protected final Writer writer;
    protected MappingStore map;
    protected HashMap<Integer, Integer> mapping = new HashMap<>();
    protected HashMap<Integer, Action> id2acts = new HashMap<>();
    protected HashMap<Integer, Action> updates = new HashMap<>();
    protected HashMap<Integer, Action> deletes = new HashMap<>();
    protected HashMap<Integer, Action> moves = new HashMap<>();
    protected HashMap<Integer, Action> inserts = new HashMap<>();
    protected Boolean isSrc;

    protected DetailedDotFormatter(Writer w, TreeContext ctx, MappingStore m ,
    		List<Action> acts, Boolean Ifsrc) {
        super(ctx);
        writer = w;
        isSrc = Ifsrc;
        map = m;
        for(Mapping map : map) {
        	ITree src = map.getFirst();
        	ITree dst = map.getSecond();
        	mapping.put(src.getId(), dst.getId());
        }
        System.out.println("mapSize:"+mapping.size());
        System.out.println("ActionSize:" + acts.size());
        for (Action a : acts) {
            ITree node = a.getNode();
            id2acts.put(node.getId(), a);
            if (a instanceof Move) {
            	moves.put(node.getId(), a);
            } else if (a instanceof Update) {
            	updates.put(node.getId(), a);
            } else if (a instanceof Insert) {
            	inserts.put(node.getId(), a);
            } else if (a instanceof Delete) {
            	deletes.put(node.getId(), a);
            }
        }
    }

    public String formate(String label) {
        if (label.contains("\"") || label.contains("\\s"))
            label = label.replaceAll("\"", "").replaceAll("\\s", "").replaceAll("\\\\", "");
        if (label.contains("\"") || label.contains("\\n"))
        	label = label.replaceAll("\"", "").replaceAll("(\r\n|\n)", "");
        if (label.length() > 30)
            label = label.substring(0, 30);
        return label;
    }

    @Override
    public void startSerialization() throws Exception {
        writer.write("digraph G {\n");
    }

    @Override
    public void startTree(ITree tree) throws Exception {
    	int id = tree.getId();
        String label = id+" "+context.getTypeLabel(tree);
        if (tree.hasLabel())
        	label = label+":"+tree.getLabel();
        label = formate(label);
        Action a = id2acts.get(id);
        int start_line = tree.getLine();
        int end_line = tree.getLastLine();
        int start_col = tree.getColumn();
        int end_col = tree.getLastColumn();
        String location = String.valueOf(start_line)+","+String.valueOf(end_line)+","+
        		String.valueOf(start_col)+","+String.valueOf(end_col);

        if(isSrc) {       	
        	if(mapping.get(id)!=null){ 
        		if (a instanceof Delete) {
        			writer.write(id + " [label=\"" + label+ "\\n"+location+
        					"\", style=dashed,"+ " color=red];\n");      					
        		}else if (a instanceof Update) {
        			Update upt = (Update)a;
        			writer.write(id + " [label=\"" + label+"\\n update to " +formate(upt.getValue())
        			+"\\n"+location+ "\", color=red];\n");
        		}else if (a instanceof Move) {          			        				
                    Move mov = (Move)a;
                    int parId = mov.getParent().getId();
                    int pos = mov.getPosition();
                    if(id!=mov.getNode().getId())
                    	throw new Exception("not equal mvoe id");
                    writer.write(id + " [label=\"" + label+"\\n move to "+parId+","+pos
        			+"\\n"+location+ "\", color=red];\n");       		
                }else       		
        			writer.write(id + " [label=\"" + label +"\\n"+location+ "\", color=red];\n");
        	}else if(mapping.get(id)==null){
        		if (a instanceof Delete) {
        			writer.write(id + " [label=\"" + label+ "\\n"+location+ "\", style=dashed];\n");
        		}else
        			writer.write(id + " [label=\"" + label + "\\n"+location+ "\", color=red];\n");
        	}
        }else if(mapping.containsValue(id)) {
        	if (a instanceof Insert) {
                Insert ins = (Insert)a;
                int parId = ins.getParent().getId();
                int pos = ins.getPosition();
                if(id!=ins.getNode().getId())
                	throw new Exception("not equal mvoe id");
                writer.write(id + " [label=\"" + label+"\\n insert to "+parId+","+pos
            			+"\\n"+location+ "\", color=red];\n");    
            }else
        	    writer.write(id + " [label=\"" + label +"\\n"+location+ "\", color=red];\n");
        }else {
        	if (a instanceof Insert) {
                Insert ins = (Insert)a;
                int parId = ins.getParent().getId();
                int pos = ins.getPosition();
                if(id!=ins.getNode().getId())
                	throw new Exception("not equal mvoe id");
                writer.write(id + " [label=\"" + label+"\\n insert to "+parId+","+pos
            			+"\\n"+location+ "\", color=red];\n");    
            }else
        	    writer.write(id + " [label=\"" + label +"\\n"+location+ "\"];\n");
        }      	

        if (tree.getParent() != null)
            writer.write(tree.getParent().getId() + " -> " + id + ";\n");
    }

    @Override
    public void stopSerialization() throws Exception {
        writer.write("}");
    }
}
