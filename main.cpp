#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <utility>
#include <tuple>
#include <iomanip>
#include <stdlib.h>
#include <chrono>
#include "./src/Graph.h"
#include "./src/Node.h"

using namespace std;

Graph *leitura(ifstream &input_file, int directed, int weightedEdge, int weightedNode)
{

    //Variáveis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;

    //Pegando a ordem do grafo
    input_file >> order;

    //Criando objeto grafo
    Graph *graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo

    if (!graph->getWeightedEdge() && !graph->getWeightedNode())
    {

        while (input_file >> idNodeSource >> idNodeTarget)
        {

            graph->insertEdge(idNodeSource, idNodeTarget, 0);
        }
    }
    else if (graph->getWeightedEdge() && !graph->getWeightedNode())
    {

        float edgeWeight;

        while (input_file >> idNodeSource >> idNodeTarget >> edgeWeight)
        {
            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
        }
    }
    else if (graph->getWeightedNode() && !graph->getWeightedEdge())
    {

        float nodeSourceWeight, nodeTargetWeight;

        while (input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight)
        {

            graph->insertEdge(idNodeSource, idNodeTarget, 0);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);
        }
    }
    else if (graph->getWeightedNode() && graph->getWeightedEdge())
    {

        float nodeSourceWeight, nodeTargetWeight, edgeWeight;

        while (input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight)
        {

            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);
        }
    }

    return graph;
}

Graph *leituraInstancia(ifstream &input_file, int directed, int weightedEdge, int weightedNode)
{

    //Variáveis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;
    int numEdges;

    //Pegando a ordem do grafo
    input_file >> order >> numEdges;

    //Criando objeto grafo
    Graph *graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo
    while (input_file >> idNodeSource >> idNodeTarget)
    {

        graph->insertEdge(idNodeSource, idNodeTarget, 0);
    }

    return graph;
}

int menu()
{

    int selecao;

    cout << "MENU" << endl;
    cout << "----" << endl;
    cout << "[1] Fecho transitivo direto de um vértice" << endl;
    cout << "[2] Caminho Mínimo entre dois vértices - Dijkstra" << endl;
    cout << "[3] Caminho Mínimo entre dois vértices - Floyd" << endl;
    cout << "[4] Árvore Geradora Mínima de Prim" << endl;
    cout << "[5] Árvore Geradora Mínima de Kruskal" << endl;
    cout << "[6] Imprimir caminhamento em Profundidade" << endl;
    cout << "[7] Imprimir ordenacao topológica" << endl;
    cout << "[8] Fecho transitivo indireto de um vértice" << endl;
    cout << "[0] Sair" << endl;

    cin >> selecao;

    return selecao;
}

void selecionar(int selecao, Graph *graph, ofstream &output_file)
{
    switch (selecao)
    {

        //Fecho transitivo direto de um vértice X;
    case 1:
    {
        if (graph->getDirected())
        {
            int no;
            cout << "Node id: ";
            cin >> no;

            if (graph->searchNode(no))
            {
                string outputData = "digraph G {" + graph->DirectTransitiveClosing(no);
                outputData = outputData.substr(0, outputData.size()-2);
                outputData = outputData + ";}";
                output_file << outputData;
            }
            else
            {
                cout << "Vertice escolhido não existe!" << endl;
            }
        }
        else
        {
            cout << "Esse algoritmo não pode ser usado nesse grafo" << endl;
        }

        break;
    }
        //Caminho mínimo entre dois vértices usando Dijkstra;
    case 2:
    {
        int source, target;
        cout << "Source id: ";
        cin >> source;
        cout << "Target id: ";
        cin >> target;

        if (graph->searchNode(target) && graph->searchNode(source))
        {
            string outputData = graph->dijkstra(source, target);
            outputData = outputData.substr(0, outputData.size()-2);
            outputData = "digraph G {" + outputData + ";}";
            output_file << outputData;
            break;
        }
        else
        {
            cout << "Um dos vertices escolhidos não existe!" << endl;
        }

        break;
    }

        //Caminho mínimo entre dois vértices usando Floyd;
    case 3:
    {
        int source, target;
        cout << "Source id: ";
        cin >> source;
        cout << "Target id: ";
        cin >> target;

        if (graph->searchNode(target) && graph->searchNode(source))
        {
            float outputData = graph->floydWarshall(source, target);
            output_file << "Caminho Mínimo entre dois vértices - Floyd: " << outputData;
        }
        else
        {
            cout << "Um dos vertices escolhidos não existe!" << endl;
        }

        break;
    }

        //AGM - Prim;
    case 4:
    {
        if (!graph->getDirected())
        {
            string outputData = graph->agmPrim();
            output_file << outputData;
            break;
        }
        else
        {
            cout << "Esse algoritmo não pode ser usado nesse grafo" << endl;
        }
    }

        //AGM - Kruskal;
    case 5:
    {
        if (!graph->getDirected())
        {
            string outputData =  "digraph G {" +  graph->agmKruskal();
            outputData = outputData.substr(0, outputData.size()-3);
            outputData = outputData + ";}";
            output_file << outputData;
            output_file << outputData;
            break;
        }
        else
        {
            cout << "Esse algoritmo não pode ser usado nesse grafo" << endl;
        }

    }

        //Busca em Profundidade;
    case 6:
    {
        int vertice;
        cout << "vertice id: ";
        cin >> vertice;

        if (graph->searchNode(vertice))
        {
            graph->buscaProfundidade(vertice);
            float outputData = 1;
            output_file << "Busca em Profundidade: " << outputData;
        }
        else
        {
            cout << "Um dos vertices escolhidos não existe!" << endl;
        }
        break;
    }
        //Ordenação Topologica;
    case 7:
    {

        break;
    }
        //Fecho transitivo indireto de um vértice X;
    case 8:
    {
        if (graph->getDirected())
        {
            int no;
            cout << "Node id: ";
            cin >> no;

            if (graph->searchNode(no))
            {
                string outputData = "digraph G {" + graph->IndirectTransitiveClosing(no);
                outputData = outputData.substr(0, outputData.size()-2);
                outputData = outputData + ";}";
                output_file << outputData;
            }
            else
            {
                cout << "Vertice escolhido não existe!" << endl;
            }
        }
        else
        {
            cout << "Esse algoritmo não pode ser usado nesse grafo" << endl;
        }
        break;
    }
    default:
    {
        cout << " Error!!! invalid option!!" << endl;
    }
    }
}

int mainMenu(ofstream &output_file, Graph *graph)
{

    int selecao = 1;

    while (selecao != 0)
    {
        // system("clear");
        selecao = menu();

        if (output_file.is_open())
            selecionar(selecao, graph, output_file);

        else
            cout << "Unable to open the output_file" << endl;

        output_file << endl;
    }

    return 0;
}

int main(int argc, char const *argv[])
{

    //Verificação se todos os parâmetros do programa foram entrados
    if (argc != 6)
    {

        cout << "ERROR: Expecting: ./<program_name> <input_file> <output_file> <directed> <weighted_edge> <weighted_node> " << endl;
        return 1;
    }

    string program_name(argv[0]);
    string input_file_name(argv[1]);

    string instance;
    if (input_file_name.find("v") <= input_file_name.size())
    {
        string instance = input_file_name.substr(input_file_name.find("v"));
        cout << "Running " << program_name << " with instance " << instance << " ... " << endl;
    }

    //Abrindo arquivo de entrada
    ifstream input_file;
    ofstream output_file;
    input_file.open(argv[1], ios::in);
    output_file.open(argv[2], ios::out | ios::trunc);

    Graph *graph;

    if (input_file.is_open())
    {

        graph = leitura(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
    }
    else
        cout << "Unable to open " << argv[1];

    mainMenu(output_file, graph);

    //Fechando arquivo de entrada
    input_file.close();

    //Fechando arquivo de saída
    output_file.close();

    return 0;
}