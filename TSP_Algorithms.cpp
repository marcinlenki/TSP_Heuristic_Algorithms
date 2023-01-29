#include <iomanip>
#include <chrono>
#include <random>
#include "TSP_Algorithms.h"

// declaring static members
double TSP_Algorithms::timeoutValue = 10;
volatile bool TSP_Algorithms::timedOut;

TSP_Algorithms::TSP_Algorithms() {
    srand(time(nullptr));
}

TSP_Algorithms::Move::Move(int ver1Index, int ver2Index, int cost) : ver_1_index(ver1Index), ver_2_index(ver2Index),
                                                                     cost(cost) {}
TSP_Algorithms::Move::Move() {}

TSP_Algorithms::Move::~Move() = default;

void TSP_Algorithms::tabuSearch(const Graph *graph, int startVertex) {
    initialize(graph, startVertex);

    tabuMatrix = new int* [numOfVertices];
    for (int i = 0; i < numOfVertices; i++)
        tabuMatrix[i] = new int [numOfVertices];

    for (int i = 0; i < numOfVertices; i++) {
        for (int j = 0; j < numOfVertices; j++) {
            tabuMatrix[i][j] = 0;
        }
    }
    TABU_TENURE = numOfVertices;

    int iterationsWithoutPositiveChange = 0;

    // Best global solution
    bestPath = vector<int>(numOfVertices + 1);
    bestPathLength = generatePath(); // modifies bestPath

    cout << "Ścieżka początkowa: " << endl;
    for (int i = 0; i < numOfVertices + 1; i++)
        cout << bestPath[i] << "  ";

    cout << endl << "Długość znalezionej ścieżki = " << bestPathLength << endl;

    // Best local solution
    vector<int> bestCandidate = bestPath;
    int bestCandidateLength = bestPathLength;

    // start measuring time on a separate thread
    thread waitThread(wait, timeoutValue);

    auto start = chrono::high_resolution_clock::now();
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);

    int timeMes = (int)timeoutValue * 1000000 / 6;
    auto start2 = chrono::high_resolution_clock::now();
    auto stop2 = chrono::high_resolution_clock::now();
    auto duration2 = chrono::duration_cast<chrono::microseconds>(stop2 - start2);

    // search for the best solution for {timeout} seconds
    while(!timedOut) {
        stop2 = chrono::high_resolution_clock::now();
        duration2 = chrono::duration_cast<chrono::microseconds>(stop2 - start2);

        if (duration2.count() >= timeMes) {
            cout << bestPathLength << endl;
            start2 = chrono::high_resolution_clock::now();
        }

        // Diversification
        // If no positive change occurs in {MAX_ITERATIONS_WITHOUT_POSITIVE_CHANGE}, shuffle current path
        if (iterationsWithoutPositiveChange == MAX_ITERATIONS_WITHOUT_POSITIVE_CHANGE) {
            shufflePath(bestCandidate, bestCandidateLength);
            iterationsWithoutPositiveChange = 0;
        }

        bool found = false;
        auto neighbourhood = getNeighbourhood(bestCandidate);

        while (!found) {
            bool aspiration = false;
            Move* bestMove = neighbourhood.top();
            neighbourhood.pop();

            int costOfCurrentMove = bestMove->cost, v1I = bestMove->ver_1_index, v2I = bestMove->ver_2_index, v1, v2;
            v1 = min(bestCandidate[v1I], bestCandidate[v2I]);
            v2 = max(bestCandidate[v1I], bestCandidate[v2I]);

            vector<int> tempLocal = bestCandidate;
            swap(tempLocal[v1I], tempLocal[v2I]);
            int tempLocalLength = bestCandidateLength + costOfCurrentMove;

            if (tempLocalLength < bestPathLength) {
                // Remember when the best solution was found
                stop = chrono::high_resolution_clock::now();
                duration = chrono::duration_cast<chrono::microseconds>(stop - start);
                bestPath = tempLocal;
                bestPathLength = tempLocalLength;

                // Aspiration
                if (tabuMatrix[v1][v2] > 0)
                    aspiration = true;

                iterationsWithoutPositiveChange = 0;
            }
            else if (tabuMatrix[v1][v2] > 0 && !neighbourhood.empty()) {
                delete bestMove;
                continue;
            }

            if (costOfCurrentMove >= 0)
                iterationsWithoutPositiveChange++;

            decrementTabuMatrix();
            bestCandidate = tempLocal;
            bestCandidateLength = tempLocalLength;
            tabuMatrix[v1][v2] = TABU_TENURE;
            if (!aspiration)
                tabuHelperList.push_back({v1, v2});

            found = true;

            // delete unused moves
            auto neighbourSize = neighbourhood.size();
            for (int i = 0; i < neighbourSize; i++) {
                Move* m = neighbourhood.top();
                neighbourhood.pop();
                delete m;
            }
        }
    }
    solutionFoundTime = (double)duration.count() / 1000000.0;

    cout << "Ścieżka: " << endl;
    for (int i = 0; i < numOfVertices + 1; i++)
        cout << bestPath[i] << "  ";

    cout << endl << "Długość znalezionej ścieżki = " << bestPathLength << endl;
    cout << "Najlepsze rozwiązanie znalezione po: " << setprecision(4) << (double)duration.count() / 1000000.0 << " sekundach" << endl;

    for (int i = 0; i < numOfVertices; i++)
        delete [] tabuMatrix[i];

    delete [] tabuMatrix;
    waitThread.join();
}

void TSP_Algorithms::simulatedAnnealing(const Graph *graph, int startVertex) {
    initialize(graph, startVertex);

    // Best global solution
    bestPath = vector<int>(numOfVertices + 1);
    bestPathLength = generatePath(); // modifies bestPath

    cout << "Ścieżka początkowa: " << endl;
    for (int i = 0; i < numOfVertices + 1; i++)
        cout << bestPath[i] << "  ";

    cout << endl << "Długość znalezionej ścieżki = " << bestPathLength << endl;

    // SA algorithm doesn't guarantee that the solution found after the algorithm is complete, is the best solution
    // that has been visited during the whole search.
    bestPathAllTime = bestPath;
    bestLengthAllTime = bestPathLength;

    // Generate starting temperature
    int numberOfSamples = 100;
    double maxProbability = 0.98;
    initialTemp = generateInitialTemperature(numberOfSamples, maxProbability) * sqrt(timeoutValue);
    currentTemp = initialTemp;
    cout << "Temperatura początkowa: " << initialTemp << endl;

    // start measuring time on a separate thread
    thread waitThread(wait, timeoutValue);

    auto start = chrono::high_resolution_clock::now();
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);

    int timeMes = (int)timeoutValue * 1000000 / 6;
    auto start2 = chrono::high_resolution_clock::now();
    auto stop2 = chrono::high_resolution_clock::now();
    auto duration2 = chrono::duration_cast<chrono::microseconds>(stop2 - start2);

    // search for the best solution for {timeout} seconds
    while(!timedOut) {
        stop2 = chrono::high_resolution_clock::now();
        duration2 = chrono::duration_cast<chrono::microseconds>(stop2 - start2);

        if (duration2.count() >= timeMes) {
            cout << bestLengthAllTime << endl;
            start2 = chrono::high_resolution_clock::now();
        }

        int numberOfChanges = 0;
        for (int i = 0; i < 100; i++) {
            if (numberOfChanges == 10)
                break;

            Move* m = generateRandomSolution(bestPath);
            int v1I = m->ver_1_index, v2I = m->ver_2_index, costOfCurrentMove = m->cost;

            vector<int> tempLocal = bestPath;
            swap(tempLocal[v1I], tempLocal[v2I]);
            int tempLocalLength = bestPathLength + costOfCurrentMove;

            double random = ((double)rand() / (double)RAND_MAX); // uniformly distributed number in range <0, 1>

            if (costOfCurrentMove < 0) {
                bestPath = tempLocal;
                bestPathLength = tempLocalLength;
                numberOfChanges++;

                if (bestPathLength < bestLengthAllTime) {
                    bestPathAllTime = bestPath;
                    bestLengthAllTime = bestPathLength;
                    stop = chrono::high_resolution_clock::now();
                    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
                }

            } else if (calculateProbability(bestPathLength, tempLocalLength, currentTemp) > random) {
                bestPath = tempLocal;
                bestPathLength = tempLocalLength;
            }

            delete m;
        }

        currentTemp *= coolingRate;
    }
    solutionFoundTime = (double)duration.count() / 1000000.0;

    cout << "Ścieżka: " << endl;
    for (int i = 0; i < numOfVertices + 1; i++)
        cout << bestPathAllTime[i] << "  ";

    cout << endl << "Długość znalezionej ścieżki = " << bestLengthAllTime << endl;
    cout << "Najlepsze rozwiązanie znalezione po: " << setprecision(4) << (double)duration.count() / 1000000.0 << " sekundach" << endl;
    cout << "Ostateczna temperatura: " << currentTemp << endl;

    waitThread.join();
}

int TSP_Algorithms::generatePath() {
    bool visited [numOfVertices];
    int currentVertex = s, bestWeight = INT_MAX, bestVertex, counter = 0, tour = 0;

    for (int i = 0; i < numOfVertices; i++)
        visited[i] = false;

    bestPath[counter] = currentVertex;

    for (int i = 0; i < numOfVertices - 1; i++) {
        visited[currentVertex] = true;

        for (int j = 0; j < numOfVertices; j++) {
            if (adjMatrix[currentVertex][j] >= 0 && !visited[j]) {
                if (adjMatrix[currentVertex][j] < bestWeight) {
                    bestWeight = adjMatrix[currentVertex][j];
                    bestVertex = j;

                }
            }
        }
        currentVertex = bestVertex;
        bestPath[++counter] = currentVertex;
        tour += bestWeight;
        bestWeight = INT_MAX;
    }

    bestPath[numOfVertices] = s;
    tour += adjMatrix[currentVertex][s];

    return tour;
}

priority_queue<TSP_Algorithms::Move*, vector<TSP_Algorithms::Move*>, TSP_Algorithms::CompareMove> TSP_Algorithms::getNeighbourhood(const vector<int> &path) const {
    int prev, next, curr, old, change;
    priority_queue<Move*, vector<Move*>, CompareMove> neighbourhood;

    // generate all possible swaps between 2 vertices (without fist and last vertex in a tour)
    // number of combinations:
    // (n - 1)! / 2(n - 3)!
    // n -> number of vertices
    for (int i = 1; i < numOfVertices - 1; i++) {
        for (int j = i + 1; j < numOfVertices; j++) {
            prev = path[i - 1];
            curr = path[j];
            old = path[i];
            next = path[i + 1];

            if (j == i + 1)
                change = (adjMatrix[prev][curr] - adjMatrix[prev][old]) + (adjMatrix[curr][old] - adjMatrix[old][next]);
            else
                change = (adjMatrix[prev][curr] - adjMatrix[prev][old]) + (adjMatrix[curr][next] - adjMatrix[old][next]);


            prev = path[j - 1];
            curr = path[i];
            old = path[j];
            next = path[j + 1];

            if (j == i + 1)
                change += (adjMatrix[curr][next] - adjMatrix[old][next]);
            else
                change += (adjMatrix[prev][curr] - adjMatrix[prev][old]) + (adjMatrix[curr][next] - adjMatrix[old][next]);

            Move* m = new Move(i, j, change);
            neighbourhood.push(m);
        }
    }

    return neighbourhood;
}

TSP_Algorithms::Move *TSP_Algorithms::generateRandomSolution(const vector<int> &path) const {
    auto avIndexes = (path.size() - 2);
    int v1I = path[(rand() % avIndexes) + 1], v2I,
            prev, next, curr, old, change;

    do {
        v2I = path[(rand() % avIndexes) + 1];
    } while (v1I == v2I);

    if (v1I > v2I)
        swap(v1I, v2I);

    prev = path[v1I - 1];
    curr = path[v2I];
    old = path[v1I];
    next = path[v1I + 1];

    if (v2I == v1I + 1) {
        change = (adjMatrix[prev][curr] - adjMatrix[prev][old]) + (adjMatrix[curr][old] - adjMatrix[old][next]);
    } else {
        change = (adjMatrix[prev][curr] - adjMatrix[prev][old]) + (adjMatrix[curr][next] - adjMatrix[old][next]);
    }

    prev = path[v2I - 1];
    curr = path[v1I];
    old = path[v2I];
    next = path[v2I + 1];

    if (v2I == v1I + 1) {
        change += (adjMatrix[curr][next] - adjMatrix[old][next]);
    }
    else {
        change += (adjMatrix[prev][curr] - adjMatrix[prev][old]) + (adjMatrix[curr][next] - adjMatrix[old][next]);
    }

    Move* m = new Move(v1I, v2I, change);
    return m;
}

double TSP_Algorithms::generateInitialTemperature(int numberOfSamples, double maxProbability) const {
    double sum = 0.0, avg;
    Move* randomMove;

    for (int i = 0; i < numberOfSamples; i++) {
        randomMove = generateRandomSolution(bestPath);
        sum += randomMove->cost;
        delete randomMove;
    }
    avg = sum / numberOfSamples;
    return -avg / (log(maxProbability));
}

double TSP_Algorithms::calculateProbability(int l1, int l2, double temp) {
    // l1 < l2
    return exp((l1 - l2) / temp);
}

void TSP_Algorithms::wait(int timeout) {
    this_thread::sleep_for(chrono::seconds(timeout));
    timedOut = true;
}

void TSP_Algorithms::decrementTabuMatrix() {
    int eraseIndex = -1;

    for (int i = 0; i < tabuHelperList.size(); i++) {
        int v1 = (tabuHelperList[i])[0];
        int v2 = (tabuHelperList[i])[1];
        tabuMatrix[v1][v2]--;

        if (tabuMatrix[v1][v2] == 0)
            eraseIndex = i;
    }

    if (eraseIndex != -1)
        tabuHelperList.erase(tabuHelperList.begin() + eraseIndex);

}

void TSP_Algorithms::shufflePath(vector<int> &path, int &newPathLength) const {
    shuffle(path.begin() + 1, path.end() - 1, mt19937(random_device()()));
    int length = 0;

    for (int i = 0; i < path.size() - 1; i++) {
        int ver1 = path[i], ver2 = path[i + 1];
        length += adjMatrix[ver1][ver2];
    }

    length += adjMatrix[path[path.size() - 1]][s];
    newPathLength = length;
}

void TSP_Algorithms::initialize(const Graph* graph, int startVertex) {
    timedOut = false;
    adjMatrix = graph->A;
    numOfVertices = graph->verticesNum;
    s = startVertex;
}

void TSP_Algorithms::setTimeoutValue(double timeoutValueArg) {
    TSP_Algorithms::timeoutValue = timeoutValueArg;
}

void TSP_Algorithms::setCoolingRate(double coolingRateArg) {
    TSP_Algorithms::coolingRate = coolingRateArg;
}