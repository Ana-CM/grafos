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
    adj = new list<iPair>[this->order + 1];
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

    adj[id].push_back(make_pair(target_id, weight));
    adj[target_id].push_back(make_pair(id, weight));
    bp[id].push_back(target_id);
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
                    if (i == idSource && j == idTarget)
                    {
                        cout << i << " -> " << k << " -> " << j;
                    }
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
        *retorno = to_string(vertice) + "->"; //imprimindo o caminho minimo
        return;                               //saindo da funcao
    }

    //chamando a funcao passando com o parametros o vetor de anteriores
    // e o vertice anterior ao atual q a funcao recebeu
    caminhoMinimo(anterior, anterior[vertice], retorno);

    *retorno += to_string(vertice) + "->"; //imprimindo o caminho minimo
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
    string response = "";
    list<Node *> listNodes;

    AuxDirectTransitiveClosing(this->getNode(no), listNodes, no);

    for (list<Node *>::iterator it = listNodes.begin(); it != listNodes.end(); it++)
    {
        Node *aux = *it;

        response += to_string(aux->getId()) + "->";
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
    vector<pair<int, iPair> >::iterator it;
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
    int i = 0;
    list<Node *> listNodes;

    AuxIndirectTransitiveClosing(this->getNode(no), listNodes, no);

    for (list<Node *>::iterator it = listNodes.begin(); it != listNodes.end(); it++)
    {
        Node *aux = *it;

        response += to_string(aux->getId()) + "->";
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
    vector<pair<int, iPair> >::iterator it;
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
    response = "";
    weight = 0;
    order = this->getOrder();
    parent = new int[order + 1];
    rank = new int[order + 1];

    for (int i = 0; i <= order; i++)
    {
        rank[i] = 0;   // Inicialmente, todos os vértices estão em conjuntos diferentes e têm classificação 0.
        parent[i] = i; // Todo vertice é pai de si mesmo
    }

    vector<pair<int, iPair> >::iterator it;
    for (it = this->edges.begin(); it != this->edges.end(); it++)
    {
        int node_1 = it->second.first;
        int node_2 = it->second.second;

        int set_node_1 = findParent(node_1, parent);
        int set_node_2 = findParent(node_2, parent);

        //vericando se existe um ciclo entre as duas arestas
        if (set_node_1 != set_node_2)
        {
            response += to_string(node_1) + " -> " + to_string(node_2) + " -> ";

            weight += it->first;

            if (rank[set_node_1] > rank[set_node_2])
                parent[set_node_2] = set_node_1;
            else
                parent[set_node_1] = set_node_2;

            if (rank[set_node_1] == rank[set_node_2])
                rank[set_node_2]++;
        }
    }

    return response;
}

list<int> *Graph::topologicalSorting()
{
    list<int> *resultado = new list<int>;
    if (!this->directed)
    {
        return nullptr;
    }
    // pilha com todos os vértices de grau 0 do Grafo
    stack<Node *> S;
    stack<pair<int, Edge *> > allEdges;
    list<int> topologicalSort;
    int nodesArray[this->order + 1];
    for (Node *currentNode = this->first_node; currentNode != this->last_node; currentNode = currentNode->getNextNode())
    {
        int currentNodeId = currentNode->getId();
        nodesArray[currentNodeId] = 0;
        // se o nó atual for for de grau zero, ele é adicionado na fila
        if (currentNode->getFirstEdge() == nullptr)
        {
            S.push(currentNode);
        }
        else
        {

            for (Edge *edge = currentNode->getFirstEdge(); edge != nullptr && edge != currentNode->getLastEdge(); edge = edge->getNextEdge())
            {
                allEdges.push(make_pair(currentNodeId, edge));
                nodesArray[currentNodeId]++;
            }
        }
    }
    int adjMatrix[this->order + 1][this->order + 1];
    for (Node *current = S.top(); !S.empty(); S.pop(), current = S.top())
    {
        topologicalSort.push_back(current->getId());
        Edge *edge = current->getFirstEdge();
        while (edge != nullptr)
        {
            adjMatrix[current->getId()][edge->getTargetId()] = 1;
            nodesArray[edge->getTargetId()]--;
            if (nodesArray[edge->getTargetId()] == 0)
            {
                topologicalSort.push_front(edge->getTargetId());
            }
            edge = edge->getNextEdge();
        }
    }
    for (pair<int, Edge *> currentEdge = allEdges.top(); !allEdges.empty(); S.pop(), currentEdge = allEdges.top())
    {
        if (adjMatrix[currentEdge.first, currentEdge.second->getTargetId()] == 0)
        {
            return nullptr;
        }
    }
    for (list<int>::iterator idNode = topologicalSort.begin(); idNode != topologicalSort.end(); idNode++)
    {
        cout << *idNode << " -> ";
    }
    return &topologicalSort;
}

string Graph::agmPrim()
{
    int order, origem;
    string response;

    //iniciando as variaveis
    response = " ";
    order = this->getOrder();                                 // Obtenha o número de vértices no gráfico
    origem = this->getFirstNode()->getId();                   // Tomando o vértice 0 como origem
    priority_queue<iPair, vector<iPair>, greater<iPair> > pq; // Cria uma fila de prioridade para armazenar vértices
    vector<int> key(order, INF);                              // Crie um vetor para as chaves e inicialize todos as chaves como infinito (INF)
    vector<int> parent(order + 1, -1);                        // Para armazenar vetor pai que ira armazenar o MST
    vector<bool> mst(order + 1, false);                       // Para acompanhar os vértices incluídos no MST

    // Insere origem na fila de prioridade e inicializa sua chave como 0.
    pq.push(make_pair(0, origem));
    key[origem] = 0;

    while (!pq.empty())
    {
        // O primeiro vértice do par é a chave mínima do vértice, extrair da fila de prioridade.
        int u = pq.top().second;
        pq.pop();

        // Podem existir valores de chave diferentes para o mesmo vértice na fila de prioridade.
        // Aquele com o menor valor de chave é sempre processado primeiro. Sendo assim, ignora o resto.
        if (mst[u] == true)
        {
            continue;
        }

        mst[u] = true;

        // 'it' é usado para obter todos os vértices adjacentes de um vértice
        list<pair<int, int> >::iterator it;
        for (it = adj[u].begin(); it != adj[u].end(); ++it)
        {
            // Obter valor do vértice e peso do adjacente atual de 'u'.
            int v = (*it).first;
            int weight = (*it).second;

            // Se v não estiver no MST e o peso for menor do que a chave atual de v, tualiza a chave de v
            if (mst[v] == false && key[v] > weight)
            {
                // Atualizando chave de v
                key[v] = weight;
                pq.push(make_pair(key[v], v));
                parent[v] = u;
            }
        }
    }

    // Imprimi o MST usando o vetor pai

    int it = 1;
    for (int i = 1; i < order + 1; i++)
    {
        it = parent[i];
        if (parent[i] == -1)
        {
            continue;
        }
        else
        {
            response += " " + to_string(it) + "->" + to_string(i);
        }
    }
    return response;
}

string Graph::buscaProfundidade(int idSource)
{
    // Marque o nó atual como visitado e imprime
    string response;
    visited[idSource] = true;
    response += to_string(idSource) + "->";

    // Recursao para todos os vértices adjacentes
    list<int>::iterator i;

    for (i = bp[idSource].begin(); i != bp[idSource].end(); ++i)
        if (!visited[*i])
            buscaProfundidade(*i);
    return response;
}