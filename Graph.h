/**************************************************************************************************
 * Implementation of the TAD Graph
**************************************************************************************************/

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED
#include "Node.h"
#include <fstream>
#include <stack>
#include <list>
#include <map>
#include <vector>

using namespace std;

class Graph
{

    //Atributes
private:
    int order;
    int number_edges;
    bool directed;
    bool weighted_edge;
    bool weighted_node;
    Node *first_node;
    Node *last_node;

public:
    //Constructor
    Graph(int order, bool directed, bool weighted_edge, bool weighted_node);
    //Destructor
    ~Graph();
    //Getters
    int getOrder();
    typedef pair<int, int> iPair;
    vector<pair<int, iPair> > edges;
    list<pair<int, int> > *adj;
    map<int, bool> visited;
    map<int, list<int> > bp;

    int getNumberEdges();
    bool getDirected();
    bool getWeightedEdge();
    bool getWeightedNode();
    Node *getFirstNode();
    Node *getLastNode();
    //Other methods
    void insertNode(int id);
    void insertEdge(int id, int target_id, float weight);
    void removeNode(int id);
    bool searchNode(int id);
    Node *getNode(int id);

    //methods phase1
    list<int> *topologicalSorting();
    string agmKruskal();
    string agmPrim();
    string buscaProfundidade(int idSource);
    float floydWarshall(int idSource, int idTarget);
    string dijkstra(int idSource, int idTarget);
    string DirectTransitiveClosing(int no);
    string IndirectTransitiveClosing(int no);
    //methods phase1
    float greed();
    float greedRandom();
    float greedRactiveRandom();
    int findParent(int aux_node, int *parent);
    void caminhoMinimo(int anterior[], int vertice, string *retorno);
    void AuxDirectTransitiveClosing(Node *no, list<Node *> &listNodes, int node_user);
    void AuxIndirectTransitiveClosing(Node *no, list<Node *> &listNodes, int node_user);

private:
    //Auxiliar methods
};

#endif // GRAPH_H_INCLUDED