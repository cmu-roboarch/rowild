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
#include <string>

// Base Node
class Node {
  public:
    virtual bool run() = 0;
};

// Action Nodes
class GoToChargingStation : public Node {
  public:
    bool run() override {
        std::cout << "Going to charging station." << std::endl;
        return true;
    }
};

class EmptyDustbin : public Node {
  public:
    bool run() override {
        std::cout << "Emptying dustbin." << std::endl;
        return true;
    }
};

class FindDirtyArea : public Node {
  public:
    bool run() override {
        std::cout << "Finding dirty area." << std::endl;
        return true;
    }
};

class CleanCurrentArea : public Node {
  public:
    bool run() override {
        std::cout << "Cleaning current area." << std::endl;
        return true;
    }
};

// Condition Nodes
class IsBatteryLow : public Node {
  public:
    bool run() override { return false; }
};

class IsDustbinFull : public Node {
  public:
    bool run() override { return false; }
};

// Composite Nodes
class Selector : public Node {
    std::vector<Node *> children;

  public:
    Selector(std::initializer_list<Node *> nodes) : children(nodes) {}
    bool run() override {
        for (Node *child : children) {
            if (child->run()) {
                return true;
            }
        }
        return false;
    }
};

class Sequence : public Node {
    std::vector<Node *> children;

  public:
    Sequence(std::initializer_list<Node *> nodes) : children(nodes) {}
    bool run() override {
        for (Node *child : children) {
            if (!child->run()) {
                return false;
            }
        }
        return true;
    }
};
