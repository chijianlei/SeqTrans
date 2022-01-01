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
 * Copyright 2015-2016 Georg Dotzler <georg.dotzler@fau.de>
 * Copyright 2015-2016 Marius Kamp <marius.kamp@fau.de>
 */
package gumtreediff.matchers;

import gumtreediff.matchers.heuristic.cd.ChangeDistillerBottomUpMatcher;
import gumtreediff.matchers.heuristic.cd.ChangeDistillerLeavesMatcher;
import gumtreediff.matchers.heuristic.cd.ChangeDistillerParallelLeavesMatcher;
import gumtreediff.matchers.heuristic.gt.GreedyBottomUpMatcher;
import gumtreediff.matchers.heuristic.gt.GreedySubtreeMatcher;
import gumtreediff.matchers.optimal.rted.RtedMatcher;
import gumtreediff.matchers.optimizations.CrossMoveMatcherThetaF;
import gumtreediff.matchers.optimizations.IdenticalSubtreeMatcherThetaA;
import gumtreediff.matchers.optimizations.InnerNodesMatcherThetaD;
import gumtreediff.matchers.optimizations.LcsOptMatcherThetaB;
import gumtreediff.matchers.optimizations.LeafMoveMatcherThetaE;
import gumtreediff.matchers.optimizations.UnmappedLeavesMatcherThetaC;
import gumtreediff.tree.ITree;

public class OptimizedVersions {

    public static class CdabcdefSeq extends CompositeMatcher {

        /**
         * Instantiates the sequential ChangeDistiller version with Theta A-F.
         *
         * @param src the src
         * @param dst the dst
         * @param store the store
         */
        public CdabcdefSeq(ITree src, ITree dst, MappingStore store) {
            super(src, dst, store,
                    new Matcher[] { new IdenticalSubtreeMatcherThetaA(src, dst, store),
                            new ChangeDistillerLeavesMatcher(src, dst, store),
                            new ChangeDistillerBottomUpMatcher(src, dst, store),
                            new LcsOptMatcherThetaB(src, dst, store),
                            new UnmappedLeavesMatcherThetaC(src, dst, store),
                            new InnerNodesMatcherThetaD(src, dst, store),
                            new LeafMoveMatcherThetaE(src, dst, store),
                            new CrossMoveMatcherThetaF(src, dst, store) });
        }
    }

    public static class CdabcdefPar extends CompositeMatcher {

        /**
         * Instantiates the parallel ChangeDistiller version with Theta A-F.
         *
         * @param src the src
         * @param dst the dst
         * @param store the store
         */
        public CdabcdefPar(ITree src, ITree dst, MappingStore store) {
            super(src, dst, store,
                    new Matcher[] { new IdenticalSubtreeMatcherThetaA(src, dst, store),
                            new ChangeDistillerParallelLeavesMatcher(src, dst, store),
                            new ChangeDistillerBottomUpMatcher(src, dst, store),
                            new LcsOptMatcherThetaB(src, dst, store),
                            new UnmappedLeavesMatcherThetaC(src, dst, store),
                            new InnerNodesMatcherThetaD(src, dst, store),
                            new LeafMoveMatcherThetaE(src, dst, store),
                            new CrossMoveMatcherThetaF(src, dst, store)

                    });
        }
    }

    public static class Gtbcdef extends CompositeMatcher {

        /**
         * Instantiates GumTree with Theta B-F.
         *
         * @param src the src
         * @param dst the dst
         * @param store the store
         */
        public Gtbcdef(ITree src, ITree dst, MappingStore store) {
            super(src, dst, store,
                    new Matcher[] { new GreedySubtreeMatcher(src, dst, store),
                            new GreedyBottomUpMatcher(src, dst, store),
                            new LcsOptMatcherThetaB(src, dst, store),
                            new UnmappedLeavesMatcherThetaC(src, dst, store),
                            new InnerNodesMatcherThetaD(src, dst, store),
                            new LeafMoveMatcherThetaE(src, dst, store),
                            new CrossMoveMatcherThetaF(src, dst, store) });
        }
    }

    public static class Rtedacdef extends CompositeMatcher {

        /**
         * Instantiates RTED with Theta A-F.
         *
         * @param src the src
         * @param dst the dst
         * @param store the store
         */
        public Rtedacdef(ITree src, ITree dst, MappingStore store) {
            super(src, dst, store,
                    new Matcher[] { new IdenticalSubtreeMatcherThetaA(src, dst, store),
                            new RtedMatcher(src, dst, store),
                            new LcsOptMatcherThetaB(src, dst, store),
                            new UnmappedLeavesMatcherThetaC(src, dst, store),
                            new InnerNodesMatcherThetaD(src, dst, store),
                            new LeafMoveMatcherThetaE(src, dst, store),
                            new CrossMoveMatcherThetaF(src, dst, store)

                    });
        }
    }

}
