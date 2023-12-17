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

#include <iostream>
#include <list>
#include <string>
#include <unordered_map>
#include <unordered_set>

using std::list;
using std::ostream;
using std::string;
using std::unordered_map;
using std::unordered_set;

class GroundedCondition {
  public:
    GroundedCondition(string predicate, list<string> argValues,
                      bool truth = true) {
        this->predicate = predicate;
        this->truth = truth;
        for (string l : argValues)
            this->argValues.push_back(l);
    }

    GroundedCondition(const GroundedCondition &gc) {
        this->predicate = gc.predicate;
        this->truth = gc.truth;
        for (string l : gc.argValues)
            this->argValues.push_back(l);
    }

    GroundedCondition getNegated() const {
        return GroundedCondition(this->predicate, this->argValues,
                                 !this->truth);
    }

    friend ostream &operator<<(ostream &os, const GroundedCondition &pred) {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition &rhs) const {
        if (this->predicate != rhs.predicate ||
            this->argValues.size() != rhs.argValues.size()) {
            return false;
        }

        auto lhsIterator = this->argValues.begin();
        auto rhsIterator = rhs.argValues.begin();

        while (lhsIterator != this->argValues.end() &&
               rhsIterator != rhs.argValues.end()) {
            if (*lhsIterator != *rhsIterator) return false;
            ++lhsIterator;
            ++rhsIterator;
        }

        if (this->truth != rhs.getTruth()) return false;

        return true;
    }

    string toString() const {
        string temp = "";
        if (!this->truth) temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->argValues)
            temp += l + ",";
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }

    string getPredicate() const { return this->predicate; }

    list<string> getArgValues() const { return this->argValues; }

    bool getTruth() const { return this->truth; }

  private:
    string predicate;
    list<string> argValues;
    bool truth = true;
};

struct GroundedConditionComparator {
    bool operator()(const GroundedCondition &lhs,
                    const GroundedCondition &rhs) const {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher {
    size_t operator()(const GroundedCondition &gcond) const {
        return std::hash<string>{}(gcond.toString());
    }
};

class Condition {
  public:
    Condition(string pred, list<string> args, bool truth) {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
            this->args.push_back(ar);
    }

    GroundedCondition *
    groundTheCondition(unordered_map<string, string> const &dict) const {
        list<string> argVals;

        for (string arg : this->args) {
            if (dict.find(arg) == dict.end()) {
                // Not provided in the action arguments, e.g., Table
                argVals.push_back(arg);
            } else {
                argVals.push_back(dict.at(arg));
            }
        }

        return new GroundedCondition(this->predicate, argVals, this->truth);
    }

    friend ostream &operator<<(ostream &os, const Condition &cond) {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition &rhs) const {
        if (this->predicate != rhs.predicate ||
            this->args.size() != rhs.args.size()) {
            return false;
        }

        auto lhsIterator = this->args.begin();
        auto rhsIterator = rhs.args.begin();

        while (lhsIterator != this->args.end() &&
               rhsIterator != rhs.args.end()) {
            if (*lhsIterator != *rhsIterator) return false;
            ++lhsIterator;
            ++rhsIterator;
        }

        if (this->truth != rhs.getTruth()) return false;

        return true;
    }

    string toString() const {
        string temp = "";
        if (!this->truth) temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
            temp += l + ",";
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }

    string getPredicate() const { return this->predicate; }
    list<string> getArgs() const { return this->args; }
    bool getTruth() const { return this->truth; }

  private:
    string predicate;
    list<string> args;
    bool truth;
};

struct ConditionComparator {
    bool operator()(const Condition &lhs, const Condition &rhs) const {
        return lhs == rhs;
    }
};

struct ConditionHasher {
    size_t operator()(const Condition &cond) const {
        return std::hash<string>{}(cond.toString());
    }
};

typedef unordered_set<Condition, ConditionHasher, ConditionComparator>
    Conditions;

class Action {
  public:
    Action(string name, list<string> args, Conditions &preconditions,
           const Conditions &effects) {
        this->name = name;
        for (string l : args)
            this->args.push_back(l);
        for (Condition pc : preconditions)
            this->preconditions.insert(pc);
        for (Condition pc : effects)
            this->effects.insert(pc);
    }

    int getAddedLiterals() const {
        int num = 0;
        for (Condition cond : effects) {
            if (cond.getTruth()) num++;
        }
        return num;
    }

    bool operator==(const Action &rhs) const {
        if (this->getName() != rhs.getName() ||
            this->getArgs().size() != rhs.getArgs().size()) {
            return false;
        }

        return true;
    }

    friend ostream &operator<<(ostream &os, const Action &ac) {
        os << ac.toString() << std::endl;
        os << "Precondition: ";
        for (Condition precond : ac.getPreconditions())
            os << precond;
        os << std::endl;
        os << "Effect: ";
        for (Condition effect : ac.getEffects())
            os << effect;
        os << std::endl;
        return os;
    }

    string toString() const {
        string temp = "";
        temp += this->getName();
        temp += "(";
        for (string l : this->getArgs())
            temp += l + ",";
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }

    string getName() const { return this->name; }
    list<string> getArgs() const { return this->args; }
    Conditions getPreconditions() const { return this->preconditions; }
    Conditions getEffects() const { return this->effects; }

  private:
    string name;
    list<string> args;
    Conditions preconditions;
    Conditions effects;
};

struct ActionComparator {
    bool operator()(const Action &lhs, const Action &rhs) const {
        return lhs == rhs;
    }
};

struct ActionHasher {
    size_t operator()(const Action &ac) const {
        return std::hash<string>{}(ac.getName());
    }
};

typedef unordered_set<GroundedCondition, GroundedConditionHasher,
                      GroundedConditionComparator>
    State;

class GroundedAction {
  public:
    GroundedAction(string name, list<string> argValues) {
        this->name = name;
        for (string ar : argValues)
            this->argValues.push_back(ar);
    }

    GroundedAction(string _name, list<string> _argValues,
                   const State &_preconditions, const State &_effects) {
        this->name = _name;
        for (string ar : _argValues) {
            this->argValues.push_back(ar);
        }
        for (GroundedCondition pcond : _preconditions) {
            this->preconditions.insert(pcond);
        }
        for (GroundedCondition effect : _effects) {
            this->effects.insert(effect);
        }
    }

    void printAction() const {
        // For debug purposes
        std::cout << "GroundedAction:" << std::endl;
        std::cout << "name: " << this->name << std::endl;
        std::cout << "argValues: ";
        for (string ar : this->argValues) {
            std::cout << ar << " ";
        }
        std::cout << std::endl << "preconditions: ";
        for (GroundedCondition gc : this->preconditions) {
            std::cout << gc << " ";
        }
        std::cout << std::endl << "effects: ";
        for (GroundedCondition gc : this->effects) {
            std::cout << gc << " ";
        }
        std::cout << std::endl << std::endl;
    }

    bool operator==(const GroundedAction &rhs) const {
        if (this->name != rhs.name ||
            this->argValues.size() != rhs.argValues.size()) {
            return false;
        }

        auto lhsIterator = this->argValues.begin();
        auto rhsIterator = rhs.argValues.begin();

        while (lhsIterator != this->argValues.end() &&
               rhsIterator != rhs.argValues.end()) {
            if (*lhsIterator != *rhsIterator) return false;
            ++lhsIterator;
            ++rhsIterator;
        }

        return true;
    }

    friend ostream &operator<<(ostream &os, const GroundedAction &gac) {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->argValues)
            temp += l + ",";
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }

    string getName() const { return this->name; }
    list<string> getArgValues() const { return this->argValues; }
    State getPreconditions() const { return this->preconditions; }
    State getEffects() const { return this->effects; }

  private:
    string name;
    list<string> argValues;
    State preconditions;
    State effects;
};

struct GroundedActionComparator {
    bool operator()(const GroundedAction &lhs,
                    const GroundedAction &rhs) const {
        return lhs == rhs;
    }
};

struct GroundedActionHasher {
    size_t operator()(const GroundedAction &ac) const {
        return std::hash<string>{}(ac.getName());
    }
};
