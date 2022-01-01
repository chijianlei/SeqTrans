package split;

import gumtreediff.tree.ITree;

public class PruneTuple {

	public final ITree par;
	public final ITree child;
	public final int position;

	public PruneTuple(ITree par, ITree child, int pos) {
		this.par = par;
		this.child = child;
		this.position = pos;
	}

	@Override
	public String toString(){
        return "("+par.getId()+","+child.getId()+","+position+")";
    }

	public ITree getPar() {
		return par;
	}

	public ITree getChild() {
		return child;
	}

	public int getPosition() {
		return position;
	}
}
