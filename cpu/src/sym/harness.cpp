/*
 * MIT License
 *
 * Copyright (c) 2023 Carnegie Mellon University
 *
 * This file is part of RoWild.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "args.h"
#include "env.h"
#include "preamble.h"
#include "zsim_hooks.h"
#include <limits>
#include <list>
#include <queue>
#include <string>
#include <vector>

list<GroundedAction> plan(Environment *env, int aStarWeight) {
    State startState = env->getStartConds();
    State goalState = env->getGoalConds();
    int maxAddedLiterals = env->getMaxAddedLiterals();

    auto getHeuristic = [maxAddedLiterals, goalState,
                         aStarWeight](State const &s) {
        int diff = 0;
        for (GroundedCondition gCond : goalState) {
            if (s.find(gCond) == s.end()) diff++;
        }
        return static_cast<double>(diff) / maxAddedLiterals * aStarWeight;
    };

    auto isGoalState = [goalState](State const &s) {
        for (GroundedCondition gCond : goalState) {
            if (s.find(gCond) == s.end()) return false;
        }
        return true;
    };

    auto convertStateToString = [](State const &s) {
        string temp = "";
        for (GroundedCondition gc : s)
            temp += gc.toString();
        return temp;
    };

    std::unordered_map<string, bool> visitedStates;
    std::unordered_map<string, int> gVals;

    auto isVisited = [&convertStateToString, &visitedStates](State const &s) {
        string key = convertStateToString(s);
        if (visitedStates.find(key) == visitedStates.end()) {
            return false;
        } else {
            return true;
        }
    };

    auto makeVisited = [&convertStateToString, &visitedStates](State const &s) {
        string key = convertStateToString(s);
        visitedStates.insert(std::make_pair(key, true));
    };

    auto getGVal = [&convertStateToString, &gVals](State const &s) {
        string key = convertStateToString(s);
        if (gVals.find(key) == gVals.end()) {
            return std::numeric_limits<int>::max();
        } else {
            return gVals[key];
        }
    };

    auto updateGVal = [&convertStateToString, &gVals](State const &s, int val) {
        string key = convertStateToString(s);
        if (gVals.find(key) == gVals.end()) {
            gVals.insert(std::make_pair(key, val));
        } else {
            gVals[key] = val;
        }
    };

    auto canApplyAction = [](State const &s, GroundedAction const &ga) {
        for (GroundedCondition precond : ga.getPreconditions()) {
            if (s.find(precond) == s.end()) {
                return false;
            }
        }
        return true;
    };

    auto applyAction = [](State const &s, GroundedAction const &ga) {
        State newState = State(s);

        for (GroundedCondition effect : ga.getEffects()) {
            if (effect.getTruth()) {
                newState.insert(effect);
            } else {
                GroundedCondition negatedEffect = effect.getNegated();
                assert((newState.find(negatedEffect) != newState.end()));
                newState.erase(negatedEffect);
            }
        }

        return newState;
    };

    struct Node {
        State state;
        int g;
        double h;
        double f;
        Node *parent;
        GroundedAction *parentAct;

        Node(State _state, int _g, double _h, Node *_parent,
             GroundedAction *_parentAct) {
            this->state = _state;
            this->g = _g;
            this->h = _h;
            this->f = g + h;
            this->parent = _parent;
            this->parentAct = _parentAct;
        }
    };

    struct Node_Comparator {
        bool operator()(const Node *left, const Node *right) {
            return left->f > right->f;
        }
    };

    std::priority_queue<Node *, std::vector<Node *>, Node_Comparator> heap;

    Actions allGroundedActions = env->getAllGroundedActs();
    list<GroundedAction> finalActions;

    Node *startNode = new Node(startState, 0 /*g*/, getHeuristic(startState),
                               NULL /*parent*/, NULL /*parentAct*/);
    heap.push(startNode);

    while (!heap.empty()) {
        Node *expNode = heap.top();
        heap.pop();

        if (isVisited(expNode->state) &&
            expNode->g >= getGVal(expNode->state)) {
            continue;
        }

        makeVisited(expNode->state);

        if (unlikely(isGoalState(expNode->state))) {
            Node *n = expNode;
            while (n->parent != NULL) {
                finalActions.push_back(*(n->parentAct));
                n = n->parent;
            }
            break;
        }

        for (GroundedAction g_act : allGroundedActions) {
            if (!canApplyAction(expNode->state, g_act)) continue;
            State newState = applyAction(expNode->state, g_act);
            int newCost = expNode->g + 1;
            if (newCost < getGVal(newState)) {
                updateGVal(newState, newCost);

                heap.push(new Node(
                    newState, newCost, getHeuristic(newState), expNode,
                    new GroundedAction(g_act.getName(), g_act.getArgValues())));
            }
        }
    }

    finalActions.reverse();
    return finalActions;
}

int main(int argc, const char **argv) {
    using args::KVArg;
    using args::Parser;

    Parser parser(argv[0], argc, argv, false);
    KVArg<std::string> inputArg(parser, "input", "", "Input problem");
    KVArg<int> aStarWeightArg(parser, "weight", "", "A* weight");
    KVArg<std::string> outputPathArg(parser, "output", "", "Output path file");

    if (!parser.parse()) assert(false);

    assert_msg(inputArg.found(), "Input problem file is not provided");

    const char *inputFile = inputArg.value().c_str();
    int aStarWeight = aStarWeightArg.found() ? aStarWeightArg.value() : 1;
    const char *outputFile =
        outputPathArg.found() ? outputPathArg.value().c_str() : "/dev/null";

    Environment *env = new Environment(inputFile);

    // ROI begin
    zsim_roi_begin();

    std::list<GroundedAction> actions = plan(env, aStarWeight);

    zsim_roi_end();
    // ROI end

    // Output the final path
    std::ofstream pathFile;
    pathFile.open(outputFile);
    for (GroundedAction gac : actions)
        pathFile << gac << std::endl;
    pathFile.close();

    return 0;
}
