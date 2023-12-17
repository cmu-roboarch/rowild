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

#pragma once

#include "log.h"
#include "preamble.h"
#include <fstream>
#include <list>
#include <string>
#include <unordered_set>

typedef unordered_set<GroundedAction, GroundedActionHasher,
                      GroundedActionComparator>
    Actions;

class Environment {
  public:
    explicit Environment(const char *fileName);
    State getStartConds() const { return startConds; }
    State getGoalConds() const { return goalConds; }
    Actions getAllGroundedActs() const;
    int getMaxAddedLiterals() const;

    friend ostream &operator<<(ostream &os, const Environment &w);

  private:
    void getPerms(list<list<string>> *combs, list<string> prefix,
                  int length) const;
    void removeStartCond(GroundedCondition gc) { this->startConds.erase(gc); }
    void addStartCond(GroundedCondition gc) { this->startConds.insert(gc); }
    void addGoalCond(GroundedCondition gc) { this->goalConds.insert(gc); }
    void removeGoalCond(GroundedCondition gc) { this->goalConds.erase(gc); }
    void addSymbol(string symbol) { symbols.insert(symbol); }
    void addAction(Action action) { this->actions.insert(action); }
    unordered_set<string> getSymbols() const { return this->symbols; }
    list<list<string>> getAllSymCombs(int length) const;
    Action getAction(string name) const;

    void addSymbols(list<string> symbols) {
        for (string l : symbols)
            this->symbols.insert(l);
    }

    State startConds, goalConds;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;
};

ostream &operator<<(ostream &os, const Environment &w);
