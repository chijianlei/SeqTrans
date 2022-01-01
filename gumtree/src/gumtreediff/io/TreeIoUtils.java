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
 * Copyright 2011-2015 Jean-Rémy Falleri <jr.falleri@gmail.com>
 * Copyright 2011-2015 Floréal Morandat <florealm@gmail.com>
 */

package gumtreediff.io;

import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.StringWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.regex.Pattern;

import javax.xml.namespace.QName;
import javax.xml.stream.XMLEventReader;
import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLOutputFactory;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamWriter;
import javax.xml.stream.events.Attribute;
import javax.xml.stream.events.EndElement;
import javax.xml.stream.events.StartElement;
import javax.xml.stream.events.XMLEvent;

import com.google.gson.stream.JsonWriter;

import gumtreediff.actions.model.Action;
import gumtreediff.actions.model.Delete;
import gumtreediff.actions.model.Insert;
import gumtreediff.actions.model.Move;
import gumtreediff.actions.model.Update;
import gumtreediff.gen.Register;
import gumtreediff.gen.TreeGenerator;
import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;
import gumtreediff.tree.TreeContext.MetadataSerializers;
import gumtreediff.tree.TreeContext.MetadataUnserializers;
import gumtreediff.tree.TreeUtils;

public final class TreeIoUtils {

    private TreeIoUtils() {} // Forbids instantiation of TreeIOUtils

    public static TreeGenerator fromXml() {
        return new XmlInternalGenerator();
    }

    public static TreeGenerator fromXml(MetadataUnserializers unserializers) {
        XmlInternalGenerator generator = new XmlInternalGenerator();
        generator.getUnserializers().addAll(unserializers);
        return generator;
    }

    public static TreeSerializer toXml(TreeContext ctx) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializers, Writer writer)
                    throws XMLStreamException {
                return new XmlFormatter(writer, ctx);
            }
        };
    }

    public static TreeSerializer toAnnotatedXml(TreeContext ctx, boolean isSrc, MappingStore m) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializers, Writer writer)
                    throws XMLStreamException {
                return new XmlAnnotatedFormatter(writer, ctx, isSrc, m);
            }
        };
    }

    public static TreeSerializer toCompactXml(TreeContext ctx) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializers, Writer writer)
                    throws Exception {
                return new XmlCompactFormatter(writer, ctx);
            }
        };
    }

    public static TreeSerializer toJson(TreeContext ctx) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializers, Writer writer)
                    throws Exception {
                return new JsonFormatter(writer, ctx);
            }
        };
    }

    public static TreeSerializer toLisp(TreeContext ctx) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializers, Writer writer)
                    throws Exception {
                return new LispFormatter(writer, ctx);
            }
        };
    }

    public static TreeSerializer toDot(TreeContext ctx, MappingStore m, List<Action> acts, Boolean Ifsrc) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializer, Writer writer)
                    throws Exception {
                return new DotFormatter(writer, ctx, m, acts, Ifsrc);
            }
        };
    }
    
    public static TreeSerializer toDetailedDot(TreeContext ctx, MappingStore m, List<Action> acts, Boolean Ifsrc) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializer, Writer writer)
                    throws Exception {
                return new DetailedDotFormatter(writer, ctx, m, acts, Ifsrc);
            }
        };
    }

    public static TreeSerializer toDot2(TreeContext ctx) {
        return new TreeSerializer(ctx) {
            @Override
            protected TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializer, Writer writer)
                    throws Exception {
                return new DotFormatter2(writer, ctx);
            }
        };
    }

    public abstract static class AbstractSerializer {

        public abstract void writeTo(Writer writer) throws Exception;

        public void writeTo(OutputStream writer) throws Exception {
            // FIXME Since the stream is already open, we should not close it, however due to semantic issue
            // it should stay like this
            try (OutputStreamWriter os = new OutputStreamWriter(writer, "UTF-8")) {
                writeTo(os);
            }
        }

        @Override
        public String toString() {
            try (StringWriter s = new StringWriter()) {
                writeTo(s);
                return s.toString();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        public void writeTo(String file) throws Exception {
            try (Writer w = Files.newBufferedWriter(Paths.get(file), Charset.forName("UTF-8"))) {
                writeTo(w);
            }
        }

        public void writeTo(File file) throws Exception {
            try (Writer w = Files.newBufferedWriter(file.toPath(), Charset.forName("UTF-8"))) {
                writeTo(w);
            }
        }
    }

    public abstract static class TreeSerializer extends AbstractSerializer {
        final TreeContext context;
        final MetadataSerializers serializers = new MetadataSerializers();

        public TreeSerializer(TreeContext ctx) {
            context = ctx;
            serializers.addAll(ctx.getSerializers());
        }

        protected abstract TreeFormatter newFormatter(TreeContext ctx, MetadataSerializers serializers, Writer writer)
                throws Exception;

        @Override
        public void writeTo(Writer writer) throws Exception {
            TreeFormatter formatter = newFormatter(context, serializers, writer);
            try {
                writeTree(formatter, context.getRoot());
            } finally {
                formatter.close();
            }
        }

        private void forwardException(Exception e) {
            throw new FormatException(e);
        }

        protected void writeTree(TreeFormatter formatter, ITree root) throws Exception {
            formatter.startSerialization();
            writeAttributes(formatter, context.getMetadata());
            formatter.endProlog();
            try {
                TreeUtils.visitTree(root, new TreeUtils.TreeVisitor() {

                    @Override
                    public void startTree(ITree tree) {
                        try {
                            assert tree != null;
                            formatter.startTree(tree);
                            writeAttributes(formatter, tree.getMetadata());
                            formatter.endTreeProlog(tree);
                        } catch (Exception e) {
                            forwardException(e);
                        }
                    }

                    @Override
                    public void endTree(ITree tree) {
                        try {
                            formatter.endTree(tree);
                        } catch (Exception e) {
                            forwardException(e);
                        }
                    }
                });
            } catch (FormatException e) {
                throw e.getCause();
            }
            formatter.stopSerialization();
        }

        protected void writeAttributes(TreeFormatter formatter, Iterator<Entry<String, Object>> it) throws Exception {
            while (it.hasNext()) {
                Entry<String, Object> entry = it.next();
                serializers.serialize(formatter, entry.getKey(), entry.getValue());
            }
        }

        public TreeSerializer export(String name, MetadataSerializer serializer) {
            serializers.add(name, serializer);
            return this;
        }

        public TreeSerializer export(String... name) {
            for (String n: name)
                serializers.add(n, Object::toString);
            return this;
        }
    }

    public interface TreeFormatter {
        void startSerialization() throws Exception;

        void endProlog() throws Exception;

        void stopSerialization() throws Exception;

        void startTree(ITree tree) throws Exception;

        void endTreeProlog(ITree tree) throws Exception;

        void endTree(ITree tree) throws Exception;

        void close() throws Exception;

        void serializeAttribute(String name, String value) throws Exception;
    }

    @FunctionalInterface
    public interface MetadataSerializer {
        String toString(Object object);
    }

    @FunctionalInterface
    public interface MetadataUnserializer {
        Object fromString(String value);
    }

    static class FormatException extends RuntimeException {
        private static final long serialVersionUID = 593766540545763066L;
        Exception cause;

        public FormatException(Exception cause) {
            super(cause);
            this.cause = cause;
        }

        @Override
        public synchronized Exception getCause() {
            return cause;
        }
    }

    static class TreeFormatterAdapter implements TreeFormatter {
        protected final TreeContext context;

        protected TreeFormatterAdapter(TreeContext ctx) {
            context = ctx;
        }

        public TreeFormatterAdapter(TreeContext ctx, MappingStore m) {
        	context = ctx;
		}

		@Override
        public void startSerialization() throws Exception { }

        @Override
        public void endProlog() throws Exception { }

        @Override
        public void startTree(ITree tree) throws Exception { }

        @Override
        public void endTreeProlog(ITree tree) throws Exception { }

        @Override
        public void endTree(ITree tree) throws Exception { }

        @Override
        public void stopSerialization() throws Exception { }

        @Override
        public void close() throws Exception { }

        @Override
        public void serializeAttribute(String name, String value) throws Exception { }
    }

    abstract static class AbsXmlFormatter extends TreeFormatterAdapter {
        protected final XMLStreamWriter writer;

        protected AbsXmlFormatter(Writer w, TreeContext ctx) throws XMLStreamException {
            super(ctx);
            XMLOutputFactory f = XMLOutputFactory.newInstance();
            writer = new IndentingXMLStreamWriter(f.createXMLStreamWriter(w));
        }

        @Override
        public void startSerialization() throws XMLStreamException {
            writer.writeStartDocument();
        }

        @Override
        public void stopSerialization() throws XMLStreamException {
            writer.writeEndDocument();
        }

        @Override
        public void close() throws XMLStreamException {
            writer.close();
        }
    }

    static class XmlFormatter extends AbsXmlFormatter {
        public XmlFormatter(Writer w, TreeContext ctx) throws XMLStreamException {
            super(w, ctx);
        }

        @Override
        public void startSerialization() throws XMLStreamException {
            super.startSerialization();
            writer.writeStartElement("root");
            writer.writeStartElement("context");
        }

        @Override
        public void endProlog() throws XMLStreamException {
            writer.writeEndElement();
        }

        @Override
        public void stopSerialization() throws XMLStreamException {
            writer.writeEndElement();
            super.stopSerialization();
        }

        @Override
        public void serializeAttribute(String name, String value) throws XMLStreamException {
            writer.writeStartElement(name);
            writer.writeCharacters(value);
            writer.writeEndElement();
        }

        @Override//�޸�AST�����
        public void startTree(ITree tree) throws XMLStreamException {
            writer.writeStartElement("tree");
            writer.writeAttribute("type", Integer.toString(tree.getType()));
            writer.writeAttribute("id", Integer.toString(tree.getId()));
            if (tree.hasLabel()) writer.writeAttribute("label", tree.getLabel());
            if (context.hasLabelFor(tree.getType()))
                writer.writeAttribute("typeLabel", context.getTypeLabel(tree.getType()));
            if (ITree.NO_VALUE != tree.getPos()) {
                writer.writeAttribute("pos", Integer.toString(tree.getPos()));
                writer.writeAttribute("length", Integer.toString(tree.getLength()));
            }
        }

        @Override
        public void endTree(ITree tree) throws XMLStreamException {
            writer.writeEndElement();
        }
    }

    static class XmlAnnotatedFormatter extends XmlFormatter {
        final SearchOther searchOther;

        public XmlAnnotatedFormatter(Writer w, TreeContext ctx, boolean isSrc,
                                     MappingStore m) throws XMLStreamException {
            super(w, ctx);

            if (isSrc)
                searchOther = (tree) -> m.hasSrc(tree) ? m.getDst(tree) : null;
            else
                searchOther = (tree) -> m.hasDst(tree) ? m.getSrc(tree) : null;
        }

        interface SearchOther {
            ITree lookup(ITree tree);
        }

        @Override
        public void startTree(ITree tree) throws XMLStreamException {
            super.startTree(tree);
            ITree o = searchOther.lookup(tree);

            if (o != null) {
                if (ITree.NO_VALUE != o.getPos()) {
                    writer.writeAttribute("other_pos", Integer.toString(o.getPos()));
                    writer.writeAttribute("other_length", Integer.toString(o.getLength()));
                }
            }
        }
    }

    static class XmlCompactFormatter extends AbsXmlFormatter {
        public XmlCompactFormatter(Writer w, TreeContext ctx) throws XMLStreamException {
            super(w, ctx);
        }

        @Override
        public void startSerialization() throws XMLStreamException {
            super.startSerialization();
            writer.writeStartElement("root");
        }

        @Override
        public void stopSerialization() throws XMLStreamException {
            writer.writeEndElement();
            super.stopSerialization();
        }

        @Override
        public void serializeAttribute(String name, String value) throws XMLStreamException {
            writer.writeAttribute(name, value);
        }

        @Override
        public void startTree(ITree tree) throws XMLStreamException {
            if (tree.getChildren().size() == 0)
                writer.writeEmptyElement(context.getTypeLabel(tree.getType()));
            else
                writer.writeStartElement(context.getTypeLabel(tree.getType()));
            if (tree.hasLabel())
                writer.writeAttribute("label", tree.getLabel());
        }

        @Override
        public void endTree(ITree tree) throws XMLStreamException {
            if (tree.getChildren().size() > 0)
                writer.writeEndElement();
        }
    }

    static class LispFormatter extends TreeFormatterAdapter {
        protected final Writer writer;
        protected final Pattern replacer = Pattern.compile("[\\\\\"]");
        int level = 0;

        protected LispFormatter(Writer w, TreeContext ctx) {
            super(ctx);
            writer = w;
        }

        @Override
        public void startSerialization() throws IOException {
            writer.write("((");
        }

        @Override
        public void startTree(ITree tree) throws IOException {
            if (!tree.isRoot())
                writer.write("\n");
            for (int i = 0; i < level; i ++)
                writer.write("    ");
            level ++;

            String pos = (ITree.NO_VALUE == tree.getPos() ? "" : String.format("(%d %d)",
                    tree.getPos(), tree.getLength()));

            writer.write(String.format("(%d %s %s (%s",
                            tree.getType(), protect(context.getTypeLabel(tree)), protect(tree.getLabel()), pos));
        }

        @Override
        public void endProlog() throws Exception {
            writer.append(") ");
        }

        @Override
        public void endTreeProlog(ITree tree) throws Exception {
            writer.append(") (");
        }

        @Override
        public void serializeAttribute(String name, String value) throws Exception {
            writer.append(String.format("(:%s %s) ", name, protect(value)));
        }

        protected String protect(String val) {
            return String.format("\"%s\"", replacer.matcher(val).replaceAll("\\\\$0"));
        }

        @Override
        public void endTree(ITree tree) throws IOException {
            writer.write(")");
            level --;
        }

        @Override
        public void stopSerialization() throws IOException {
            writer.write(")");
        }
    }

    static class DotFormatter extends TreeFormatterAdapter {
        protected final Writer writer;
        protected MappingStore map;
        protected HashMap<Integer, Integer> mapping = new HashMap<>();
        protected HashMap<Integer, Action> udActions = new HashMap<>();
        protected HashMap<Integer, Action> amActions = new HashMap<>();
        protected Boolean isSrc;

        protected DotFormatter(Writer w, TreeContext ctx, MappingStore m ,
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
                ITree src = a.getNode();
                if (a instanceof Move) {
                	amActions.put(src.getId(), a);
                } else if (a instanceof Update) {
                	udActions.put(src.getId(), a);
                } else if (a instanceof Insert) {
                	amActions.put(src.getId(), a);
                } else if (a instanceof Delete) {
                	udActions.put(src.getId(), a);
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

            if(isSrc) {
            	if(udActions.get(id)!=null&&mapping.get(id)!=null){
            		Action a = udActions.get(id);
            		if (a instanceof Delete) {
            			writer.write(id + " [label=\"" + label+ "\", style=dashed,"
            					+ " color=red];\n");
            		}
            		else if (a instanceof Update) {
            			Update upt = (Update)a;
            			writer.write(id + " [label=\"" + label+"\\n update to " +formate(upt.getValue())
            			+ "\", color=red];\n");
            		}else
            			writer.write(id + " [label=\"" + label + "\", color=red];\n");
            	}else if(mapping.get(id)==null){
            		Action a = udActions.get(id);
            		if (a instanceof Delete) {
            			writer.write(id + " [label=\"" + label+ "\", style=dashed];\n");
            		}
            		else if(a instanceof Update) {
            			Update upt = (Update)a;
            			writer.write(id + " [label=\"" + label+"\\n update to " +formate(upt.getValue())
            			+ "\"];\n");
            		}else
            			writer.write(id + " [label=\"" + label + "\", color=red];\n");
            	}else
            		writer.write(id + " [label=\"" + label + "\", color=red];\n");
            }
            else if(mapping.containsValue(id)) {
            	writer.write(id + " [label=\"" + label + "\", color=red];\n");
            }else
            	writer.write(id + " [label=\"" + label + "\"];\n");

            if (tree.getParent() != null)
                writer.write(tree.getParent().getId() + " -> " + id + ";\n");
//            if(isSrc==true&&actions.get(id)!=null) {
//            	Action a = actions.get(id);
//            	if (a instanceof Move) {
//                    Move mov = (Move)a;
//                    writer.write(mov.getNode().getId()+" -> "+mov.getParent().getId()
//                    		+" [style=dashed, label=\"mov "+mov.getNode().getId()
//                    		+" to "+mov.getParent().getId()+","+mov.getPosition()+"\"];\n");
//                } else if (a instanceof Insert) {
//                    Insert ins = (Insert)a;
//                    writer.write(ins.getNode().getId()+" -> "+ins.getParent().getId()
//                    		+" [style=dashed, label=\"ins "+ins.getNode().getId()
//                    		+" to "+ins.getParent().getId()+","+ins.getPosition()+"\"];\n");
//                }
//            }
        }

        @Override
        public void stopSerialization() throws Exception {
            writer.write("}");
        }
    }

    static class JsonFormatter extends TreeFormatterAdapter {
        private final JsonWriter writer;

        public JsonFormatter(Writer w, TreeContext ctx) {
            super(ctx);
            writer = new JsonWriter(w);
            writer.setIndent("  ");
        }

        @Override
        public void startTree(ITree t) throws IOException {
            writer.beginObject();
            writer.name("type").value(Integer.toString(t.getType()));
            if (t.hasLabel()) writer.name("label").value(t.getLabel());
            if (context.hasLabelFor(t.getType())) writer.name("typeLabel").value(context.getTypeLabel(t.getType()));
//            if (ITree.NO_VALUE != t.getPos()) {
//                writer.name("pos").value(Integer.toString(t.getPos()));
//                writer.name("length").value(Integer.toString(t.getLength()));
//            }
            if (t.getId()>=0) writer.name("id").value(t.getId());
        }

        @Override
        public void endTreeProlog(ITree tree) throws IOException {
            writer.name("children");
            writer.beginArray();
        }

        @Override
        public void endTree(ITree tree) throws IOException {
            writer.endArray();
            writer.endObject();
        }

        @Override
        public void startSerialization() throws IOException {
            writer.beginObject();
            writer.setIndent("\t");
        }

        @Override
        public void endProlog() throws IOException {
            writer.name("root");
        }

        @Override
        public void serializeAttribute(String key, String value) throws IOException {
            writer.name(key).value(value);
        }

        @Override
        public void stopSerialization() throws IOException {
            writer.endObject();
        }

        @Override
        public void close() throws IOException {
            writer.close();
        }
    }

    @Register(id = "xml", accept = "\\.gxml$")
    // TODO Since it is not in the right package, I'm not even sure it is visible in the registry
    // TODO should we move this class elsewhere (another package)
    public static class XmlInternalGenerator extends TreeGenerator {

        static MetadataUnserializers defaultUnserializers = new MetadataUnserializers();
        final MetadataUnserializers unserializers = new MetadataUnserializers(); // FIXME should it be pushed up or not?

        private static final QName TYPE = new QName("type");

        private static final QName LABEL = new QName("label");
        private static final QName TYPE_LABEL = new QName("typeLabel");
        private static final String POS = "pos";
        private static final String LENGTH = "length";

        static {
            defaultUnserializers.add(POS, Integer::parseInt);
            defaultUnserializers.add(LENGTH, Integer::parseInt);
        }

        public XmlInternalGenerator() {
            unserializers.addAll(defaultUnserializers);
        }

        @Override
        protected TreeContext generate(Reader source) throws IOException {
            XMLInputFactory fact = XMLInputFactory.newInstance();
            TreeContext context = new TreeContext();
            try {
                ArrayDeque<ITree> trees = new ArrayDeque<>();
                XMLEventReader r = fact.createXMLEventReader(source);
                while (r.hasNext()) {
                    XMLEvent e = r.nextEvent();
                    if (e instanceof StartElement) {
                        StartElement s = (StartElement) e;
                        if (!s.getName().getLocalPart().equals("tree")) // FIXME need to deal with options
                            continue;
                        int type = Integer.parseInt(s.getAttributeByName(TYPE).getValue());

                        ITree t = context.createTree(type,
                                labelForAttribute(s, LABEL), labelForAttribute(s, TYPE_LABEL));
                        // FIXME this iterator has no type, due to the API. We have to cast it later
                        Iterator<?> it = s.getAttributes();
                        while (it.hasNext()) {
                            Attribute a = (Attribute) it.next();
                            unserializers.load(t, a.getName().getLocalPart(), a.getValue());
                        }

                        if (trees.isEmpty())
                            context.setRoot(t);
                        else
                            t.setParentAndUpdateChildren(trees.peekFirst());
                        trees.addFirst(t);
                    } else if (e instanceof EndElement) {
                        if (!((EndElement)e).getName().getLocalPart().equals("tree")) // FIXME need to deal with options
                            continue;
                        trees.removeFirst();
                    }
                }
                context.validate();
                return context;
            } catch (Exception e) {
                e.printStackTrace();
            }
            return null;
        }

        private static String labelForAttribute(StartElement s, QName attrName) {
            Attribute attr = s.getAttributeByName(attrName);
            return attr == null ? ITree.NO_LABEL : attr.getValue();
        }

        public MetadataUnserializers getUnserializers() {
            return unserializers;
        }
    }

    static class DotFormatter2 extends TreeFormatterAdapter {
        protected final Writer writer;
        protected MappingStore map;
        protected HashMap<Integer, Integer> mapping = new HashMap<>();
        protected HashMap<Integer, Action> udActions = new HashMap<>();
        protected HashMap<Integer, Action> amActions = new HashMap<>();
        protected Boolean isSrc;

        protected DotFormatter2(Writer w, TreeContext ctx) {
            super(ctx);
            writer = w;
        }

        public String formate(String label) {
            if (label.contains("\"") || label.contains("\\s"))
                label = label.replaceAll("\"", "").replaceAll("\\s", "").replaceAll("\\\\", "");
            if (label.contains("\"") || label.contains("\\n"))
            	label = label.replaceAll("\"", "").replaceAll("(\r\n|\n)", "");
//            if (label.length() > 30)
//                label = label.substring(0, 30);
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

            writer.write(id + " [label=\"" + label + "\"];\n");


            if (tree.getParent() != null)
                writer.write(tree.getParent().getId() + " -> " + id + ";\n");
        }

        @Override
        public void stopSerialization() throws Exception {
            writer.write("}");
        }
    }
}
