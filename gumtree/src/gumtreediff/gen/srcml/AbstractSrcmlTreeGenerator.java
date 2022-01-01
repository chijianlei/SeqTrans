/*
 * This file is part of GumTree.
 *
 * GumTree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GumTree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with GumTree.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 Jean-R茅my Falleri <jr.falleri@gmail.com>
 */

package gumtreediff.gen.srcml;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.StringReader;
import java.io.Writer;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Scanner;
import java.util.Set;

import javax.xml.namespace.QName;
import javax.xml.stream.XMLEventReader;
import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.events.Characters;
import javax.xml.stream.events.EndElement;
import javax.xml.stream.events.StartElement;
import javax.xml.stream.events.XMLEvent;

import gumtreediff.gen.TreeGenerator;
import gumtreediff.io.LineReader;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public abstract class AbstractSrcmlTreeGenerator extends TreeGenerator {

    private static final String SRCML_CMD = System.getProperty("gt.srcml.path", "srcml");

    private static final QName LINE = new  QName("http://www.srcML.org/srcML/position", "line", "pos");

    private static final QName COLUMN = new  QName("http://www.srcML.org/srcML/position", "column", "pos");

    private LineReader lr;

    private Set<String> labeled = new HashSet<>(
            Arrays.asList("specifier", "name", "comment", "literal", "operator",
            		"modifier", "include", "directive", "file", "argument", "value"));//第二行gumtree本身未添加

    private StringBuilder currentLabel;

    private TreeContext context;

    @Override
    public TreeContext generate(Reader r) throws IOException {
        lr = new LineReader(r);
        String xml = getXml(lr);
        return getTreeContext(xml);
    }

    public TreeContext getTreeContext(String xml) {
        XMLInputFactory fact = XMLInputFactory.newInstance();
        context = new TreeContext();
        currentLabel = new StringBuilder();
        try {
            ArrayDeque<ITree> trees = new ArrayDeque<>();
            XMLEventReader r = fact.createXMLEventReader(new StringReader(xml));
            while (r.hasNext()) {
                XMLEvent ev = r.nextEvent();
                if (ev.isStartElement()) {
                    StartElement s = ev.asStartElement();
//                    System.out.println("StartElL:"+ev.toString());
                    String typeLabel = s.getName().getLocalPart();
//                    System.out.println("typeLabel:"+typeLabel);
                    if (typeLabel.equals("position")) {
                    	setLength(trees.peekFirst(), s);
                    }else if(typeLabel.equals("comment")) {
                    	int type = typeLabel.hashCode();
                    	ITree t = context.createTree(type, "", typeLabel);
                    	trees.addFirst(t);
                    	continue;//不需要comment节点
                    }else {
                        int type = typeLabel.hashCode();
                        ITree t = context.createTree(type, "", typeLabel);

                        if (trees.isEmpty()) {
                            context.setRoot(t);
                            t.setPos(0);
                        } else {
                            t.setParentAndUpdateChildren(trees.peekFirst());
//                            System.out.println("setpos");
                            setPos(t, s);
                        }
                        trees.addFirst(t);
                    }
                } else if (ev.isEndElement()) {
                    EndElement end = ev.asEndElement();
//                    System.out.println("ev:"+end.toString());
                    if (!end.getName().getLocalPart().equals("position")) {
                        if (isLabeled(trees))
                            trees.peekFirst().setLabel(currentLabel.toString());
                        trees.removeFirst();
                        currentLabel = new StringBuilder();
                    }
                } else if (ev.isCharacters()) {
                    Characters chars = ev.asCharacters();
                    if (!chars.isWhiteSpace() && isLabeled(trees))
                        currentLabel.append(chars.getData().trim());
                }
            }
            fixPos(context);
            context.validate();
            int size = context.getRoot().getDescendants().size();
            context.setSize(size);
            return context;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    private boolean isLabeled(ArrayDeque<ITree> trees) {
        return labeled.contains(context.getTypeLabel(trees.peekFirst().getType()));
    }

    private void fixPos(TreeContext ctx) {
        for (ITree t : ctx.getRoot().postOrder()) {
            if (!t.isLeaf()) {
                if (t.getPos() == ITree.NO_VALUE || t.getLength() == ITree.NO_VALUE) {
                    ITree firstChild = t.getChild(0);
                    t.setPos(firstChild.getPos());
                    if (t.getChildren().size() == 1)
                        t.setLength(firstChild.getLength());
                    else {
                        ITree lastChild = t.getChild(t.getChildren().size() - 1);
                        t.setLength(lastChild.getEndPos() - firstChild.getPos());
                    }
                }
            }
        }
    }

    private void setPos(ITree t, StartElement e) {
        if (e.getAttributeByName(LINE) != null) {
            int line = Integer.parseInt(e.getAttributeByName(LINE).getValue());
            int column = Integer.parseInt(e.getAttributeByName(COLUMN).getValue());
//            System.out.println("line:"+line);
//            System.out.println("column"+column);
            t.setPos(lr.positionFor(line, column));
            t.setLine(line);
            t.setColumn(column);
        }
    }

    private void setLength(ITree t, StartElement e) {
        if (t.getPos() == -1)
            return;
        if (e.getAttributeByName(LINE) != null) {
            int line = Integer.parseInt(e.getAttributeByName(LINE).getValue());
            int column = Integer.parseInt(e.getAttributeByName(COLUMN).getValue());
            t.setLength(lr.positionFor(line, column) - t.getPos() + 1);
            t.setLastLine(line);
            t.setLastColumn(column);
        }
    }

    public String getXml(Reader r) throws IOException {
//    	String path = "exception.txt";
//    	BufferedWriter wr = new BufferedWriter(new FileWriter(new File(path)));
        //FIXME this is not efficient but I am not sure how to speed up things here.
        File f = File.createTempFile("gumtree", "");
        File xmlFile = new File("raw.xml");
        BufferedWriter wr = new BufferedWriter(new FileWriter(xmlFile));
        try (
                Writer w = Files.newBufferedWriter(f.toPath(), Charset.forName("UTF-8"));

                BufferedReader br = new BufferedReader(r);
        ) {
            String line = br.readLine();
            while (line != null) {
                w.append(line + System.lineSeparator());
                line = br.readLine();
            }
        }
        ProcessBuilder pb = new ProcessBuilder(getArguments(f.getAbsolutePath()));
        pb.redirectErrorStream(true);
        pb.directory(f.getParentFile());
        try {
        	Process p = pb.start();
            Scanner scanner = new Scanner(p.getInputStream(), "UTF-8");
            StringBuilder buf = new StringBuilder();
            // TODO Why do we need to read and bufferize everything, when we could/should only use generateFromStream
            while (scanner.hasNextLine()) {
                buf.append(scanner.nextLine() + System.lineSeparator());
            }
            scanner.close();
            p.waitFor();
//            if (exit != 0) {
//            	throw new RuntimeException();
//            }
            p.destroy();
//            Thread.sleep(1000);
            String xml = buf.toString();
//            System.out.println("xml:"+xml);
            wr.append(xml);
            wr.newLine();
            wr.flush();
            return xml;
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        } finally {
        	wr.close();
            f.delete();
        }

    }

    private static void splitMessage(final InputStream input) {
//    	try  {
//            Process p = b.start();
//            final  InputStream is1 = p.getInputStream();
//            final  InputStream is2 = p.getErrorStream();
//            new  Thread() {
//                public void run(){
//                	try  {
//                    BufferedReader br = new  BufferedReader(new InputStreamReader(is1, "UTF-8"));
//                    StringBuilder buf = new StringBuilder();
//                        String line = null ;
//                        while((line = br.readLine())!=null){
//                        	buf.append(line + System.lineSeparator());
//                        }
//                      r.close();
//                      xml = buf.toString();
//                    } catch  (IOException e) {
//                        e.printStackTrace();
//                    }
//                }
//            }.start();
//
//            new  Thread() {
//                public void run(){
//                	try {
//                    BufferedReader br2 = new BufferedReader(new InputStreamReader(is2, "UTF-8"));
//                        String lineC = null ;
//                        while((lineC = br2.readLine())!= null){
//                            if(lineC!=null)
//                            	System.out.println(lineC);
//                        }
//                    } catch (IOException e) {
//                        e.printStackTrace();
//                    }
//                }
//            }.start();
//            p.waitFor();
//        } catch  (Exception e) {
//            System.err.println(e);
//        }

    	new Thread(new Runnable(){
    		@Override
			public void run() {
    			try {
    			Reader r = new InputStreamReader(input, "UTF-8");
    			BufferedReader br = new BufferedReader(r);
    			StringBuilder buf = new StringBuilder();
    			String line = null;
					while((line=br.readLine())!=null) {
						buf.append(line + System.lineSeparator());
//						System.out.println(line);
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
    		}
    	}).start();
    }

    public abstract String getLanguage();

    public String[] getArguments(String file) {
        return new String[]{SRCML_CMD, "-l", getLanguage(), "--position", file, "--tabs=1"};
    }
}
