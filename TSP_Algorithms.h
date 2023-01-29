#ifndef PROJEKT1_TSP_ALGORITHMS_H
#define PROJEKT1_TSP_ALGORITHMS_H

#include "Graph.h"
#include <vector>
#include <thread>
#include <chrono>
#include <queue>
#include <cmath>
using namespace std;

class TSP_Algorithms {
public:
    struct Move {
        int ver_1_index;
        int ver_2_index;
        int cost;

        Move();
        Move(int ver1Index, int ver2Index, int cost);
        ~Move();
    };

    struct CompareMove {
        bool operator() (Move* const& m1, Move* const& m2) {
            return m1->cost > m2->cost;
        }
    };

    // Time out value for both tabu search (TS) and simulated annealing (SW)
    static double timeoutValue;   // in seconds
    static volatile bool timedOut;
    double solutionFoundTime;

    int** adjMatrix;
    int numOfVertices;
    int s;  // start vertex

    // best tour
    int bestPathLength;
    vector<int> bestPath;

    // TS
    int** tabuMatrix;
    vector<vector<int>> tabuHelperList;
    int TABU_TENURE;
    const int MAX_ITERATIONS_WITHOUT_POSITIVE_CHANGE = 10000;

    // SW
    vector<int> bestPathAllTime;
    int bestLengthAllTime;
    double coolingRate = 0.9999;
    double initialTemp;
    double currentTemp;

    void initialize(const Graph* graph, int startVertex);
    static void wait(int timeout);
    int generatePath();

    priority_queue<Move*, vector<Move*>, CompareMove> getNeighbourhood(const vector<int> &path) const;
    void decrementTabuMatrix();
    void shufflePath(vector<int> &path, int &newPathLength) const;


    double generateInitialTemperature(int numberOfSamples, double maxProbability) const;
    double calculateProbability(int l1, int l2, double temp);
    Move* generateRandomSolution(const vector<int> &path) const;


public:
    static void setTimeoutValue(double timeoutValueArg);
    void setCoolingRate(double coolingRateArg);
    void tabuSearch(const Graph* graph, int startVertex);
    void simulatedAnnealing(const Graph* graph, int startVertex);
    TSP_Algorithms();
};

#endif