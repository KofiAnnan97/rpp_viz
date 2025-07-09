#include "map_data.hpp"
#include "pathworker.h"

#include <QElapsedTimer>
#include <QCoreApplication>

PathWorker::PathWorker(QObject *parent)
    : QObject(parent),
    m_stop_requested(false)
{
    qDebug() << "Worker created in thread:" << QThread::currentThreadId();
}

PathWorker::~PathWorker()
{
    qDebug() << "Worker destroyed in thread:" << QThread::currentThreadId();
}

// BFS algorithm module
void PathWorker::run_bfs(Graph g){
    auto bfs = BFS(g);
    auto start_time = high_resolution_clock::now();
    bfs.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto data = bfs.reconstruct_path(g.root, g.end);
    AlgoResult ar;
    ar.type = "BFS";
    ar.duration = duration_cast<milliseconds>(end_time - start_time).count();
    ar.path = data.first;
    ar.dist = data.second;
    ar.travelled = bfs.get_travelled_nodes();
    results.push_back(ar);
}

// A* algorithm module
void PathWorker::run_a_star(Graph g){
    auto as = AStar(g);
    auto start_time = high_resolution_clock::now();
    as.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto data = as.reconstruct_path(g.root, g.end);
    AlgoResult ar;
    ar.type = "A*";
    ar.duration = duration_cast<milliseconds>(end_time- start_time).count();
    ar.path = data.first;
    ar.dist = data.second;
    ar.travelled = as.get_travelled_nodes();
    results.push_back(ar);
}

// RRT* algorithm module
void PathWorker::run_rrt_star(Graph g, int px_width, int px_height, int max_iters){
    auto rrt = RRTStar(g, px_width, px_height, max_iters);
    auto start_time = high_resolution_clock::now();
    rrt.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto data = rrt.reconstruct_path(g.root, g.end);
    AlgoResult ar;
    ar.type = "RRT*";
    ar.duration = duration_cast<milliseconds>(end_time- start_time).count();
    ar.path = data.first;
    ar.dist = data.second;
    ar.travelled = rrt.get_travelled_nodes();
    results.push_back(ar);
}

// Compute path(s)
void PathWorker::compute_path(QString algo_name, Graph g, int width, int height, int max_iters){
    results.clear();
    QElapsedTimer timer;
    timer.start();
    if(algo_name == "BFS") this->run_bfs(g);
    else if(algo_name ==  "A*") this->run_a_star(g);
    else if(algo_name == "RRT*") this->run_rrt_star(g, width, height, max_iters);
    else if(algo_name =="All"){
        this->run_bfs(g);
        this->run_a_star(g);
        this->run_rrt_star(g, width, height, max_iters);
    }
    emit compute_finished(results);
}


