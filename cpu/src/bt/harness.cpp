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

#include "zsim_hooks.h"
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <thread>
#include <vector>

enum class Status { RUNNING, SUCCESS, FAILURE };

class Node {
  public:
    virtual ~Node() = default;
    virtual Status tick() = 0;
};

class CompositeNode : public Node {
  protected:
    std::vector<Node *> children;

  public:
    void addChild(Node *child) { children.push_back(child); }

    virtual ~CompositeNode() {
        for (Node *child : children) {
            delete child;
        }
    }
};

class Selector : public CompositeNode {
  public:
    Status tick() override {
        for (Node *child : children) {
            if (child->tick() == Status::SUCCESS) {
                return Status::SUCCESS;
            }
        }
        return Status::FAILURE;
    }
};

class Sequence : public CompositeNode {
  public:
    Status tick() override {
        for (Node *child : children) {
            if (child->tick() != Status::SUCCESS) {
                return Status::FAILURE;
            }
        }
        return Status::SUCCESS;
    }
};

class Leaf : public Node {
    Status (*function)();

  public:
    Leaf(Status (*func)()) : function(func) {}
    Status tick() override { return function(); }
};

bool pathIsClear = true;
bool bookFound = false;
int bookSearchAttempts = 0;

Status navigateToShelf() {
    if (pathIsClear) {
        std::cout << "Navigating to shelf...\n";

        // Simulated delay for the robot moving
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return Status::SUCCESS;
    } else {
        std::cout << "Navigation failed. Path is not clear.\n";
        return Status::FAILURE;
    }
}

Status checkPathClear() {
    pathIsClear = (rand() % 2) == 0; // 50% chance path is clear
    if (pathIsClear) {
        std::cout << "Path is clear!\n";
        return Status::SUCCESS;
    } else {
        std::cout << "Path is not clear.\n";
        return Status::FAILURE;
    }
}

Status waitForPath() {
    std::cout << "Waiting for path to clear...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return checkPathClear();
}

Status searchBook() {
    while (bookSearchAttempts < 10) {
        std::cout << "Searching for book...\n";
        bookFound =
            (rand() % 2) == 0; // 50% chance to find the book each attempt
        bookSearchAttempts++;

        if (bookFound) {
            std::cout << "Book found!\n";
            return Status::SUCCESS;
        } else {
            std::cout << "Book not found. Trying again...\n";
        }
    }
    std::cout << "Failed to find the book after multiple attempts.\n";
    return Status::FAILURE;
}

Status pickUpBook() {
    if (bookFound) {
        std::cout << "Picking up the book...\n";
        // Simulated delay for picking up the book
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return Status::SUCCESS;
    } else {
        std::cout << "No book to pick up.\n";
        return Status::FAILURE;
    }
}

Status returnToStart() {
    std::cout << "Returning to starting position...\n";
    // Simulated delay for the robot moving
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return Status::SUCCESS;
}

int main(int argc, const char **argv) {
    Sequence mainSequence;

    // ROI begin
    zsim_roi_begin();

    // Navigate to shelf sequence
    Sequence *navigateSequence = new Sequence();
    Selector *pathSelector = new Selector();

    Leaf *checkPathLeaf = new Leaf(checkPathClear);
    Leaf *navigateLeaf = new Leaf(navigateToShelf);
    Leaf *waitForPathLeaf = new Leaf(waitForPath);

    pathSelector->addChild(checkPathLeaf);
    pathSelector->addChild(waitForPathLeaf);

    navigateSequence->addChild(pathSelector);
    navigateSequence->addChild(navigateLeaf);

    // Search and pickup sequence
    Sequence *searchPickupSequence = new Sequence();
    Leaf *searchLeaf = new Leaf(searchBook);
    Leaf *pickupLeaf = new Leaf(pickUpBook);

    searchPickupSequence->addChild(searchLeaf);
    searchPickupSequence->addChild(pickupLeaf);

    // Return sequence
    Leaf *returnLeaf = new Leaf(returnToStart);

    mainSequence.addChild(navigateSequence);
    mainSequence.addChild(searchPickupSequence);
    mainSequence.addChild(returnLeaf);

    mainSequence.tick();

    zsim_roi_end();
    // ROI end

    return 0;
}
