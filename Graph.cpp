#include "Graph.h"

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

Graph::Graph() {
    verticesNum = 0;
    A = nullptr;
}

Graph::~Graph() {
    clear();
}

bool Graph::makeFromFile(const string& fileName) {
    int tempV, w;
    string tempName, tempType, tempComment, keyword;
    ifstream inFile;
    inFile.open(fileName);

    if (!inFile.is_open()) {
        cout<<"Plik nie istnieje."<<endl;
        return false;
    }

    inFile >> keyword >> tempName;
    inFile >> keyword >> tempType;
    if (tempType != "ATSP") {
        cout<<"Plik "<<tempName<<" nie odnosi sie do problemu ATSP."<<endl;
        return false;
    }

    inFile >> keyword;
    getline(inFile, tempComment);
    inFile >> keyword >> tempV;

    if (tempV <= 0) {
        cout<<"W pliku wykryto blednę dane."<<endl;
        return false;
    }

    clear();

    header.NAME = tempName;
    header.TYPE = tempType;
    header.COMMENT = tempComment;
    verticesNum = header.DIMENSION = tempV;
    inFile >> keyword >> header.EDGE_WEIGHT_TYPE;
    inFile >> keyword >> header.EDGE_WEIGHT_FORMAT;

    A = new int * [verticesNum];
    for(int i = 0; i < verticesNum; i++) {
        A[i] = new int[verticesNum]{};
    }

    do {
        inFile >> keyword;
    } while (keyword != "EDGE_WEIGHT_SECTION");

    for (int i = 0; i < verticesNum; i++) {
        for (int j = 0; j < verticesNum; j++) {
            inFile >> w;
            A[i][j] = i == j ? -1 : w;
        }
    }

    inFile>>keyword; // EOF text in file
    inFile>>keyword; // standard EOF

    if (inFile.eof()) {
        cout<<"Wczytywanie danych zakończone."<<endl;
    } else if(inFile.fail()) {
        cout<<"Wczytywanie danych przerwane, nie udało się wczytać pliku."<<endl;
        return false;
    } else {
        cout<<"Wczytywanie danych przerwane."<<endl;
        return false;
    }

    inFile.close();
    return true;
}

void Graph::clear() {
    for(int i = 0; i < verticesNum; i++) {
        for(int j = 0; j < verticesNum; j++) {
            A[i][j] = 0;
        }

        delete [] A[i];
    }

    verticesNum = 0;
    delete [] A;
}

void Graph::show() {
    for (int i = 0; i < verticesNum; i++) {
        for (int j = 0; j < verticesNum; j++) {
            cout<<setw(6)<<A[i][j];
        }
        cout<<endl;
    }
    cout<<endl;
}