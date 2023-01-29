#ifndef PROJEKT1_GRAPH_H
#define PROJEKT1_GRAPH_H
#include <iostream>
#include <string>
using namespace std;


class Graph {
public:
    // Not used in the final version
    struct FileHeader {
        string NAME;
        string TYPE;
        string COMMENT;
        int DIMENSION;
        string EDGE_WEIGHT_TYPE;
        string EDGE_WEIGHT_FORMAT;

    public:
        void printHeader() const {
            cout<<"NAME: "<<NAME<<endl;
            cout<<"TYPE: "<<TYPE<<endl;
            cout<<"COMMENT: "<<COMMENT<<endl;
            cout<<"DIMENSION: "<<DIMENSION<<endl;
            cout<<"EDGE_WEIGHT_TYPE: "<<EDGE_WEIGHT_TYPE<<endl;
            cout<<"EDGE_WEIGHT_FORMAT: "<<EDGE_WEIGHT_FORMAT<<endl;
        }
    };

    FileHeader header;
    int verticesNum;
    int** A;

    Graph();
    ~Graph();
    bool makeFromFile(const string& fileName);
    void clear();
    void show();
};

#endif