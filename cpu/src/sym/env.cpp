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

#include "env.h"
#include <list>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using std::regex;
using std::sregex_token_iterator;

Environment::Environment(const char *fileName) {
    auto parseSymbols = [](string symbolsStr) {
        list<string> symbols;
        size_t pos = 0;
        string delimiter = ",";
        while ((pos = symbolsStr.find(delimiter)) != string::npos) {
            string symbol = symbolsStr.substr(0, pos);
            symbolsStr.erase(0, pos + delimiter.length());
            symbols.push_back(symbol);
        }
        symbols.push_back(symbolsStr);
        return symbols;
    };

    std::ifstream file(fileName);

    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);

    enum class PARSER {
        Symbols,
        Initial,
        Goal,
        Actions,
        Action_Definition,
        Action_Precondition,
        Action_Effect
    };
    PARSER parser = PARSER::Symbols;

    Conditions preconditions, effects;
    string actionName, actionArgs, line;

    assert(file.good());

    while (getline(file, line)) {
        string::iterator endPos = remove(line.begin(), line.end(), ' ');
        line.erase(endPos, line.end());

        if (line == "") continue;

        if (parser == PARSER::Symbols) {
            std::smatch results;
            if (regex_search(line, results, symbolStateRegex)) {
                line = line.substr(8);
                sregex_token_iterator iter(line.begin(), line.end(),
                                           symbolRegex, 0);
                sregex_token_iterator end;

                this->addSymbols(parseSymbols(iter->str()));

                parser = PARSER::Initial;
            } else {
                panic("Symbols are not specified correctly");
            }
        } else if (parser == PARSER::Initial) {
            const char *lineC = line.c_str();
            if (regex_match(lineC, initialConditionRegex)) {
                const std::vector<int> subMatches = {1, 2};
                sregex_token_iterator iter(line.begin(), line.end(),
                                           conditionRegex, subMatches);
                sregex_token_iterator end;

                while (iter != end) {
                    // name
                    string predicate = iter->str();
                    iter++;
                    // args
                    string args = iter->str();
                    iter++;

                    if (predicate[0] == '!') {
                        this->removeStartCond(GroundedCondition(
                            predicate.substr(1), parseSymbols(args)));
                    } else {
                        this->addStartCond(
                            GroundedCondition(predicate, parseSymbols(args)));
                    }
                }

                parser = PARSER::Goal;
            } else {
                panic("Initial conditions not specified correctly");
            }
        } else if (parser == PARSER::Goal) {
            const char *lineC = line.c_str();
            if (regex_match(lineC, goalConditionRegex)) {
                const std::vector<int> subMatches = {1, 2};
                sregex_token_iterator iter(line.begin(), line.end(),
                                           conditionRegex, subMatches);
                sregex_token_iterator end;

                while (iter != end) {
                    // name
                    string predicate = iter->str();
                    iter++;
                    // args
                    string args = iter->str();
                    iter++;

                    if (predicate[0] == '!') {
                        this->removeGoalCond(GroundedCondition(
                            predicate.substr(1), parseSymbols(args)));
                    } else {
                        this->addGoalCond(
                            GroundedCondition(predicate, parseSymbols(args)));
                    }
                }

                parser = PARSER::Actions;
            } else {
                panic("Goal conditions not specified correctly");
            }
        } else if (parser == PARSER::Actions) {
            const char *lineC = line.c_str();
            if (regex_match(lineC, actionRegex)) {
                parser = PARSER::Action_Definition;
            } else {
                panic("Actions not specified correctly");
            }
        } else if (parser == PARSER::Action_Definition) {
            const char *lineC = line.c_str();
            if (regex_match(lineC, conditionRegex)) {
                const std::vector<int> subMatches = {1, 2};
                sregex_token_iterator iter(line.begin(), line.end(),
                                           conditionRegex, subMatches);
                sregex_token_iterator end;
                // name
                actionName = iter->str();
                iter++;
                // args
                actionArgs = iter->str();
                iter++;

                parser = PARSER::Action_Precondition;
            } else {
                panic("Action not specified correctly");
            }
        } else if (parser == PARSER::Action_Precondition) {
            const char *lineC = line.c_str();
            if (regex_match(lineC, precondRegex)) {
                const std::vector<int> subMatches = {1, 2};
                sregex_token_iterator iter(line.begin(), line.end(),
                                           conditionRegex, subMatches);
                sregex_token_iterator end;

                while (iter != end) {
                    // name
                    string predicate = iter->str();
                    iter++;
                    // args
                    string args = iter->str();
                    iter++;

                    bool truth;

                    if (predicate[0] == '!') {
                        predicate = predicate.substr(1);
                        truth = false;
                    } else {
                        truth = true;
                    }

                    Condition precond(predicate, parseSymbols(args), truth);
                    preconditions.insert(precond);
                }

                parser = PARSER::Action_Effect;
            } else {
                panic("Precondition not specified correctly");
            }
        } else if (parser == PARSER::Action_Effect) {
            const char *lineC = line.c_str();
            if (regex_match(lineC, effectRegex)) {
                const std::vector<int> subMatches = {1, 2};
                sregex_token_iterator iter(line.begin(), line.end(),
                                           conditionRegex, subMatches);
                sregex_token_iterator end;

                while (iter != end) {
                    // name
                    string predicate = iter->str();
                    iter++;
                    // args
                    string args = iter->str();
                    iter++;

                    bool truth;

                    if (predicate[0] == '!') {
                        predicate = predicate.substr(1);
                        truth = false;
                    } else {
                        truth = true;
                    }

                    Condition effect(predicate, parseSymbols(args), truth);
                    effects.insert(effect);
                }

                this->addAction(Action(actionName, parseSymbols(actionArgs),
                                       preconditions, effects));

                preconditions.clear();
                effects.clear();
                parser = PARSER::Action_Definition;
            } else {
                panic("Effects not specified correctly");
            }
        }
    }
    file.close();
}

void Environment::getPerms(list<list<string>> *combs, list<string> prefix,
                           int length) const {
    if (length == 0) {
        combs->push_back(prefix);
        return;
    }

    for (string sym : symbols) {
        list<string> newPrefix = prefix;
        newPrefix.push_back(sym);
        getPerms(combs, newPrefix, length - 1);
    }
}

list<list<string>> Environment::getAllSymCombs(int length) const {
    assert(length <= static_cast<int>(symbols.size()));
    list<list<string>> combs;
    getPerms(&combs, {}, length);
    return combs;
}

Action Environment::getAction(string name) const {
    for (Action a : this->actions) {
        if (a.getName() == name) return a;
    }
    panic("Action %s not found!", name.c_str());
}

Actions Environment::getAllGroundedActs() const {
    Actions allGroundedActs;

    for (Action act : this->actions) {
        string grActName = act.getName();
        list<string> actArgs = act.getArgs();
        list<list<string>> symPerms = getAllSymCombs(actArgs.size());

        for (list<string> perm : symPerms) {
            assert(perm.size() == actArgs.size());
            auto argIterator = actArgs.begin();
            auto symIterator = perm.begin();
            unordered_map<string, string> dict;
            while (argIterator != actArgs.end()) {
                assert((dict.find(*argIterator) == dict.end()));

                dict.insert(std::make_pair(*argIterator, *symIterator));
                ++argIterator;
                ++symIterator;
            }

            list<string> grActArgsVals;
            for (string arg : actArgs) {
                assert((dict.find(arg) != dict.end()));
                grActArgsVals.push_back(dict[arg]);
            }

            State grActPreconds;
            for (Condition cond : act.getPreconditions()) {
                grActPreconds.insert(*(cond.groundTheCondition(dict)));
            }

            State grActAllRawEffects;
            for (Condition cond : act.getEffects()) {
                grActAllRawEffects.insert(*(cond.groundTheCondition(dict)));
            }

            // An optimization: Remove contradictory effects: e.g., On(A, B)
            // and !On(A, B). Then if the number of effects becomes zero,
            // simply omit the action
            State grActEffectiveEffects;
            for (auto effect1 : grActAllRawEffects) {
                bool effective = true;
                for (auto effect2 : grActAllRawEffects) {
                    if (effect1 == effect2.getNegated()) {
                        effective = false;
                        break;
                    }
                }
                if (effective) grActEffectiveEffects.insert(effect1);
            }

            if (!grActEffectiveEffects.empty()) {
                allGroundedActs.insert(GroundedAction(grActName, grActArgsVals,
                                                      grActPreconds,
                                                      grActEffectiveEffects));
            }
        }
    }

    return allGroundedActs;
}

int Environment::getMaxAddedLiterals() const {
    int max = -1, count;
    for (Action ac : actions) {
        if ((count = ac.getAddedLiterals()) > max) max = count;
    }
    return max;
}

ostream &operator<<(ostream &os, const Environment &w) {
    os << "***** Environment *****" << std::endl << std::endl;
    os << "Symbols: ";
    for (string s : w.getSymbols())
        os << s + ",";
    os << std::endl;
    os << "Initial conditions: ";
    for (GroundedCondition s : w.startConds)
        os << s;
    os << std::endl;
    os << "Goal conditions: ";
    for (GroundedCondition g : w.goalConds)
        os << g;
    os << std::endl;
    os << "Actions:" << std::endl;
    for (Action g : w.actions)
        os << g << std::endl;
    std::cout << "***** Environment Created! *****" << std::endl;
    return os;
}
