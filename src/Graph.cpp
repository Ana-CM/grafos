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
#include <string>
#include<algorithm>

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
    this->edges.push_back({weight, {id, target_id}});
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
    return 0;
}

void caminhoMinimo(int anterior[], int vertice, string *retorno)
{
    if (anterior[vertice] == -1)
    {
        *retorno = to_string(vertice) + " "; //imprimindo o caminho minimo
        return;                              //saindo da funcao
    }

    //chamando a funcao passando com o parametros o vetor de anteriores
    // e o vertice anterior ao atual q a funcao recebeu
    caminhoMinimo(anterior, anterior[vertice], retorno);

    *retorno += to_string(vertice) + " "; //imprimindo o caminho minimo
}

string Graph::dijkstra(int idSource, int idTarget)
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
    previousEdges[idSource] = -1;

    priorities.push_back(make_pair(distancies[idSource], idSource));
    while (!priorities.empty())
    {
        pair<int, int> topPair = priorities.front(); // pegando o elemento do vector de priorities
        int topEdge = topPair.second;                // recuperando o vertice do topo
        //removendo o item do vector (pois seus dados já foram "guardados" para serem utilizados abaixo)
        priorities.erase(priorities.begin());
        if (!visited[topEdge]) // verificando se o vertice já foi visitado
        {
            visited[topEdge] = true;
            list<pair<int, int> >::iterator it;
            //percorrendo os vertices adjacentes ao vertice visitado
            for (Edge *it = this->getNode(topEdge)->getFirstEdge(); it != nullptr; it = it->getNextEdge())
            {
                int vertice = it->getTargetId();    //obtem o vertice
                int custo_aresta = it->getWeight(); //obtem o custo da aresta
                // verificando a menor distancia
                if (distancies[vertice] > (distancies[topEdge] + custo_aresta))
                {
                    distancies[vertice] = distancies[topEdge] + custo_aresta; // atualizando a distancia
                    previousEdges[vertice] = topEdge;                         //armazena o vertice anterior ao vertice atual

                    // inserindo na fila de priorities o vertice adjacente e a distancia
                    priorities.push_back(make_pair(distancies[vertice], vertice));
                    //         sort(priorities.begin(), priorities.end()); // Ordenado o vector pela distancia minima
                }
            }
        }
    }
    string retorno;
    caminhoMinimo(previousEdges, idTarget, &retorno);

    delete[] visited,
        distancies, previousEdges;
    return retorno;
}

string Graph::DirectTransitiveClosing(int no)
{
    return "nada";
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
    return nullptr;
}

int find(int u, int *parent)
{
    /* Make the parent of the nodes in the path
        from u--> parent[u] point to parent[u] */
    if (u != parent[u])
        parent[u] = find(parent[u], parent);
    return parent[u];
}
  

string Graph::agmKruskal()
{
    int weight, order, *parent, *rank;  
    string response;
 
    response = "Árvore Geradora Mínima de Kruskal: ";
    weight   = 0;
    order    = this->getOrder();
    parent   = new int[ order+1 ];
    rank     = new int[ order+1 ];

    for( int i = 0; i <= order; i++ )
    {
        rank[i] = 0;
        parent[i] = i;
    }
	
    sort( this->edges.begin(), this->edges.end() ); //ordenando as arestas em ordem crescente de custo

    vector< pair<int, iPair> >::iterator it;
	for ( it = this->edges.begin(); it != this->edges.end(); it++ )
	{   
        int x, y;
		int u = it->second.first;
		int v = it->second.second;

		int set_u = find(u, parent);
		int set_v = find(v, parent);

		if (set_u != set_v)
		{
			response += to_string(u) + " - " + to_string(v) + " // ";

			weight += it->first;
			
            x = find(u, parent);
            y = find(v, parent);
            if (rank[x] > rank[y])
			     parent[y] = x;
		    else  
			     parent[x] = y;

		    if (rank[x] == rank[y])
			    rank[y]++;
		}
	}

    response += "Peso: " + to_string(weight);

    return response;
}

Graph *agmPrim()
{
    return nullptr;
}