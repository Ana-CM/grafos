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
#include <algorithm>

using namespace std;
#define INF 0x3f3f3f3f
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
    adj = new list<iPair>[this->order];
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
    this->edges.push_back(make_pair(weight, make_pair(id, target_id)));
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

float Graph::floydWarshall(int idSource, int idTarget)
{
    int graphOrder = this->getOrder();

    float solution[graphOrder][graphOrder];
    for (int i = 0; i < graphOrder; i++)
    {
        for (int j = 0; j < graphOrder; j++)
        {
            if (i == j)
            {
                solution[i][j] = 0;
            }
            else
            {
                solution[i][j] = FLT_MAX;
                Node *iNode = this->getNode(i);
                Edge *edgeBetweenIJ = iNode->hasEdgeBetween(j);
                if (edgeBetweenIJ != nullptr)
                {
                    solution[i][j] = edgeBetweenIJ->getWeight();
                }
            }
        }
    }
    for (int k = 0; k < graphOrder; k++)
    {
        for (int i = 0; i < graphOrder; i++)
        {
            for (int j = 0; j < graphOrder; j++)
            {
                if (solution[i][j] > solution[i][k] + solution[k][j])
                {
                    solution[i][j] = solution[i][k] + solution[k][j];
                }
            }
        }
    }

    return solution[idSource][idTarget];
}

void Graph::caminhoMinimo(int anterior[], int vertice, string *retorno)
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
    vector<pair<int, int>> priorities; // o primeiro elemento do pair é a distancia e o segundo eh o vertice

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
            list<pair<int, int>>::iterator it;
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
    string response = "";
    list<Node *> listNodes;

    AuxDirectTransitiveClosing(this->getNode(no), listNodes, no);

    for (list<Node *>::iterator it = listNodes.begin(); it != listNodes.end(); it++)
    {
        Node *aux = *it;
        response += to_string(aux->getId()) + " ";
    }

    return response;
}

void Graph::AuxDirectTransitiveClosing(Node *no, list<Node *> &listNodes, int node_user)
{
    // Parada, pois o vertice já está na solução
    if (find(listNodes.begin(), listNodes.end(), no) != listNodes.end())
        return;

    // vértice atual é adicionado na solução
    if (no->getId() != node_user)
    {
        listNodes.push_back(no);
    }

    // Percorrendo as arestas e buscando mais vértices que podem ser atingidos
    vector<pair<int, iPair>>::iterator it;
    for (it = this->edges.begin(); it != this->edges.end(); it++)
    {
        if (it->second.first == node_user || (find(listNodes.begin(), listNodes.end(), this->getNode(it->second.first)) != listNodes.end()))
        {

            AuxDirectTransitiveClosing(this->getNode(it->second.second), listNodes, node_user);
        }
    }
}

string Graph::IndirectTransitiveClosing(int no)
{
    string response = "";
    list<Node *> listNodes;

    AuxIndirectTransitiveClosing(this->getNode(no), listNodes, no);

    for (list<Node *>::iterator it = listNodes.begin(); it != listNodes.end(); it++)
    {
        Node *aux = *it;
        response += to_string(aux->getId()) + " ";
    }

    return response;
}

void Graph::AuxIndirectTransitiveClosing(Node *no, list<Node *> &listNodes, int node_user)
{
    // Parada, pois o vertice já está na solução
    if (find(listNodes.begin(), listNodes.end(), no) != listNodes.end())
        return;

    // vértice atual é adicionado na solução
    if (no->getId() != node_user)
    {
        listNodes.push_back(no);
    }

    // Percorrendo as arestas e buscando mais vértices que podem ser atingidos
    vector<pair<int, iPair>>::iterator it;
    for (it = this->edges.begin(); it != this->edges.end(); it++)
    {
        if (it->second.second == node_user || (find(listNodes.begin(), listNodes.end(), this->getNode(it->second.second)) != listNodes.end()))
        {

            AuxIndirectTransitiveClosing(this->getNode(it->second.first), listNodes, node_user);
        }
    }
}

//Retorna o pai do vertice
int Graph::findParent(int aux_node, int *parent)
{
    if (aux_node != parent[aux_node])
        parent[aux_node] = findParent(parent[aux_node], parent);

    return parent[aux_node];
}

string Graph::agmKruskal()
{
    int weight, order, *parent, *rank;
    string response;

    //iniciando as variaveis
    response = "Árvore Geradora Mínima de Kruskal: ";
    weight = 0;
    order = this->getOrder();
    parent = new int[order + 1];
    rank = new int[order + 1];

    for (int i = 0; i <= order; i++)
    {
        rank[i] = 0;   // Inicialmente, todos os vértices estão em conjuntos diferentes e têm classificação 0.
        parent[i] = i; // Todo vertice é pai de si mesmo
    }

    vector<pair<int, iPair>>::iterator it;
    for (it = this->edges.begin(); it != this->edges.end(); it++)
    {
        int node_1 = it->second.first;
        int node_2 = it->second.second;

        int set_node_1 = findParent(node_1, parent);
        int set_node_2 = findParent(node_2, parent);

        //vericando se existe um ciclo entre as duas arestas
        if (set_node_1 != set_node_2)
        {
            response += to_string(node_1) + " - " + to_string(node_2) + " // ";

            weight += it->first;

            if (rank[set_node_1] > rank[set_node_2])
                parent[set_node_2] = set_node_1;
            else
                parent[set_node_1] = set_node_2;

            if (rank[set_node_1] == rank[set_node_2])
                rank[set_node_2]++;
        }
    }

    response += "Peso: " + to_string(weight);

    return response;
}

void Graph::topologicalSorting()
{
}
void Graph::agmPrim()
{
    int order, origem;
    string response;

    //iniciando as variaveis
    response "Árvore Geradora Mínima de Prim: ";
    order = this->getOrder();                                // Obtenha o número de vértices no gráfico
    origem = 0;                                              // Tomando o vértice 0 como origem
    priority_queue<iPair, vector<iPair>, greater<iPair>> pq; // Cria uma fila de prioridade para armazenar vértices que estão sendo preinMST.
    vector<int> key(order, INF);                             // Crie um vetor para as chaves e inicialize todos as chaves como infinito (INF)
    vector<int> parent(order, -1);                           // Para armazenar vetor pai que ira armazenar a MST
    vector<bool> inMST(order, false);                        // Para acompanhar os vértices incluídos na MST

    // Insere origem na fila de prioridade e inicializa sua chave como 0.
    pq.push(make_pair(0, origem));
    key[origem] = 0;

    while (!pq.empty())
    {
        // O primeiro vértice do par é a chave mínima vértice, extraia-o da fila de prioridade.
        // rótulo do vértice é armazenado no segundo do par (ele tem que ser feito desta forma para manter os vértices
        // chave classificada (a chave deve ser o primeiro item em pares)
        int u = pq.top().second;
        pq.pop();

        // Podem existir valores de chave diferentes para o mesmo vértice na fila de prioridade.
        // Aquele com o menor valor de chave é sempre processado primeiro. Sendo assim, ignora o resto.
        if (inMST[u] == true)
        {
            continue;
        }

        inMST[u] = true; // Include vertex in MST

        // 'it' é usado para obter todos os vértices adjacentes de um vértice
        list<pair<int, int>>::iterator it;
        for (it = adj[u].begin(); it != adj[u].end(); ++it)
        {
            // Obter valor do vértice e peso do adjacente atual de 'u'.
            int v = (*it).first;
            int weight = (*it).second;

            // Se v não estiver no MST e o peso de (u, v) for menor do que a chave atual de v, tualiza a chave de v
            if (inMST[v] == false && key[v] > weight)
            {
                // Atualizando chave de v
                key[v] = weight;
                pq.push(make_pair(key[v], v));
                parent[v] = u;
            }
        }
    }

    // Imprimir bordas de MST usando o vetor pai

    for (int i = 1; i < order; ++i)
    {
        response += to_string(parent[i]) + " - " + to_string(i);
    }
    return response;
}

void Graph::buscaProfundidade(int idSource)
{
    // Marque o nó atual como visitado e imprime
    string response;
    visited[idSource] = true;
    response += to_string(idSource) + " ";

    // Recursao para todos os vértices adjacentes para o vértice atual
    list<int>::iterator i;
    for (i = this->edges.begin(); i != this->edges.end(); ++i)
    {
        if (!visited[*i])
            buscaProfundidade(*i);
    }
    return response;
}
}