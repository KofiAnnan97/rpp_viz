#include <QCoreApplication>

#include "pathworker.h"
#include "bfs.hpp"
#include "a_star.hpp"
#include "rrt_star.hpp"

PathWorker::PathWorker(QObject *parent)
    : QObject(parent)
{
    qDebug() << "Worker created in thread:" << QThread::currentThreadId();
}

PathWorker::~PathWorker()
{
    qDebug() << "Worker destroyed in thread:" << QThread::currentThreadId();
}

void PathWorker::add_result(string algo_type, int duration, vector<cell> path, float dist, vector<cell> travelled){
    results.push_back(AlgoResult{algo_type, duration, path, travelled, dist});
}

// BFS algorithm module
void PathWorker::run_bfs(Graph g){
    auto bfs = BFS(g);
    auto start_time = high_resolution_clock::now();
    bfs.solve(g.root, g.end, compute_timeout);
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time-start_time);
    if(duration.count() >= compute_timeout) timeout_occurred = true;
    auto data = bfs.reconstruct_path(g.root, g.end);
    this->add_result(bfs_id.toStdString(), duration.count(), data.first, data.second, bfs.get_travelled_nodes());
}

// A* algorithm module
void PathWorker::run_a_star(Graph g){
    auto as = AStar(g);
    auto start_time = high_resolution_clock::now();
    as.solve(g.root, g.end, compute_timeout);
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time-start_time);
    if(duration.count() >= compute_timeout) timeout_occurred = true;
    auto data = as.reconstruct_path(g.root, g.end);
    this->add_result(a_star_id.toStdString(), duration.count(), data.first, data.second, as.get_travelled_nodes());
}

// RRT* algorithm module
void PathWorker::run_rrt_star(Graph g, int max_iters){
    auto rrt = RRTStar(g, max_iters);
    auto start_time = high_resolution_clock::now();
    rrt.solve(g.root, g.end, compute_timeout);
    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time-start_time);
    if(duration.count() >= compute_timeout) timeout_occurred = true;
    auto data = rrt.reconstruct_path(g.root, g.end);
    this->add_result(rrt_star_id.toStdString(), duration.count(), data.first, data.second, rrt.get_travelled_nodes());
}

// Compute path(s)
void PathWorker::compute_path(QString algo_name, Graph g, int max_iters){
    results.clear();
    QString err_msg;
    if(algo_name == bfs_id || algo_name == all_id){
        this->run_bfs(g);
        if(timeout_occurred){
            err_msg += QString("   - BFS Computation exceeded %1 ms\n").arg(compute_timeout);
            timeout_occurred = false;
        }
    }
    if(algo_name ==  a_star_id || algo_name == all_id){
        this->run_a_star(g);
        if(timeout_occurred){
            err_msg += QString("   - A* Computation exceeded %1 ms\n").arg(compute_timeout);
            timeout_occurred = false;
        }
    }
    if(algo_name == rrt_star_id || algo_name == all_id){
        this->run_rrt_star(g, max_iters);
        if(timeout_occurred){
            err_msg += QString("   - RRT* Computation exceeded %1 ms\n").arg(compute_timeout);
            timeout_occurred = false;
        }
    }

    if(!err_msg.isEmpty()) emit compute_error(results, err_msg);
    else emit compute_finished(results);
}
