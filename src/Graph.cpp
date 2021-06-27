#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <iomanip>
#include <vector>

using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{

    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->number_edges = 0;
}

// Destructor
Graph::~Graph()
{

    Node *next_node = this->first_node;

    while (next_node != nullptr)
    {

        next_node->removeAllEdges();
        Node *aux_node = next_node->getNextNode();
        delete next_node;
        next_node = aux_node;
    }
}

// Getters
int Graph::getOrder()
{

    return this->order;
}
int Graph::getNumberEdges()
{

    return this->number_edges;
}
//Function that verifies if the graph is directed
bool Graph::getDirected()
{

    return this->directed;
}
//Function that verifies if the graph is weighted at the edges
bool Graph::getWeightedEdge()
{

    return this->weighted_edge;
}

//Function that verifies if the graph is weighted at the nodes
bool Graph::getWeightedNode()
{

    return this->weighted_node;
}

Node *Graph::getFirstNode()
{

    return this->first_node;
}

Node *Graph::getLastNode()
{

    return this->last_node;
}

// Other methods
/*
    The outdegree attribute of nodes is used as a counter for the number of edges in the graph.
    This allows the correct updating of the numbers of edges in the graph being directed or not.
*/
void Graph::insertNode(int id)
{
    Node *No = new Node(id);
    if (this->first_node == nullptr)
    {

        this->first_node = No;
        this->last_node = No;
    }
    else
    {

        if (this->searchNode(id) == false)
        {
            this->last_node->setNextNode(No);
            this->last_node = No;
        }
    }
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    if (!searchNode(id))
    {
        this->insertNode(id);
    }
    if (!searchNode(target_id))
    {
        this->insertNode(target_id);
    }
    Node *originNode = getNode(id), *targetNode = getNode(target_id);
    if (!directed)
    {
        targetNode->insertEdge(id, weight);
    }
    originNode->insertEdge(target_id, weight);
}

void Graph::removeNode(int id)
{

    if (this->first_node != nullptr)
    {

        if (this->searchNode(id))
        {

            if (this->first_node->getId() == id)
            {

                Node *atual = this->first_node;

                atual->removeAllEdges();

                this->first_node = atual->getNextNode();

                delete atual;
            }
            else
            {

                Node *atual = this->first_node;
                Node *anterior = nullptr;

                while (atual->getId() != id)
                {
                    anterior = atual;
                    atual = atual->getNextNode();
                }

                anterior->setNextNode(atual->getNextNode());

                if (this->last_node->getId() == id)
                {

                    this->last_node = anterior;
                }

                atual->removeAllEdges();

                delete atual;
            }
        }
    }
}

bool Graph::searchNode(int id)
{
    if (this->first_node != nullptr)
    {
        Node *node = this->first_node;
        while (node->getNextNode() != nullptr)
        {

            if (node->getId() == id)
            {
                return true;
            }

            node = node->getNextNode();
        }
        if (node->getId() == id)
        {
            return true;
        }

        return false;
    }

    return false;
}

Node *Graph::getNode(int id)
{
    if (searchNode(id))
    {
        Node *node = this->first_node;

        while (node->getNextNode() != nullptr)
        {
            if (node->getId() == id)
            {
                return node;
            }

            node = node->getNextNode();
        }
        if (node->getId() == id)
        {
            return node;
        }

        return nullptr;
    }

    return nullptr;
}

//Function that prints a set of edges belongs breadth tree

void Graph::breadthFirstSearch(ofstream &output_file)
{
}

float Graph::floydMarshall(int idSource, int idTarget)
{
}

void caminhoMinimo(int anterior[], int vertice)
{
    // cout << "caminhoMinimo do " << vertice << endl;
    // cout << "anterior[vertice] " << anterior[vertice] << endl;
    // caso base (esse vertice eh o primeiro, logo nao existe um vertice anterior a ele)
    if (anterior[vertice] <= 0)
    {
        return; //saindo da funcao
    }

    //chamando a funcao passando com o parametros o vetor de anteriores
    // e o vertice anterior ao atual q a funcao recebeu
    caminhoMinimo(anterior, anterior[vertice]);

    std::cout << vertice << " "; //imprimindo o caminho minimo
}

float Graph::dijkstra(int idSource, int idTarget)
{
    int *distancies = new int[this->order];
    int *visited = new int[this->order];
    int *previousEdges = new int[this->order];
    vector<pair<int, int> > priorities; // o primeiro elemento do pair é a distancia e o segundo eh o vertice

    for (int i = 0; i < this->order; i++)
    {
        distancies[i] = 10000000;
        visited[i] = false;
    }

    //distancia da origem para a origem
    distancies[idSource] = 0;

    //caso base (não existe vertice anterior ao primeiro)
    previousEdges[0] = -1;

    priorities.push_back(make_pair(distancies[idSource], idSource));
    while (!priorities.empty())
    {
        pair<int, int> topPair = priorities.front(); // pegando o elemento do vector de priorities
        int topEdge = topPair.second;                // recuperando o vertice do topo
        //removendo o item do vector (pois seus dados já foram "guardados" para serem utilizados abaixo)
        cout << "TOPEDGE" << topEdge << endl;
        priorities.erase(priorities.begin());
        if (!visited[topEdge]) // verificando se o vertice já foi visitado
        {
            visited[topEdge] = true;
            list<pair<int, int> >::iterator it;
            //percorrendo os vertices adjacentes ao vertice visitado
            cout << "get node" << this->getNode(topEdge)->getFirstEdge() << endl;
            for (Edge *it = this->getNode(topEdge)->getFirstEdge(); it != nullptr; it = it->getNextEdge())
            {
                cout << "hello" << it << endl;
                int vertice = it->getTargetId();    //obtem o vertice
                int custo_aresta = it->getWeight(); //obtem o custo da aresta
                // verificando a menor distancia
                if (distancies[vertice] > (distancies[topEdge] + custo_aresta))
                {
                    cout << "entrei no if " << vertice << endl;
                    distancies[vertice] = distancies[topEdge] + custo_aresta; // atualizando a distancia
                    previousEdges[vertice] = topEdge;                         //armazena o vertice anterior ao vertice atual

                    // inserindo na fila de priorities o vertice adjacente e a distancia
                    priorities.push_back(make_pair(distancies[vertice], vertice));
                    //         sort(priorities.begin(), priorities.end()); // Ordenado o vector pela distancia minima
                }
            }
        }
    }
    cout << " caso base " << previousEdges[0] << endl;
    cout << "caminho minimo: ";
    // caminhoMinimo(previousEdges, idTarget);
    cout << endl
         << "distancia minima: " << distancies[idTarget];
    delete[] visited, distancies, previousEdges;
}

//function that prints a topological sorting
void topologicalSorting()
{
}

void breadthFirstSearch(ofstream &output_file)
{
}
Graph *getVertexInduced(int *listIdNodes)
{
}

Graph *agmKuskal()
{
}
Graph *agmPrim()
{
}
