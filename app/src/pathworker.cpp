#include <QCoreApplication>

#include "pathworker.h"
#include "time_helper.hpp"
#include "bfs.hpp"
#include "a_star.hpp"
#include "rrt_star.hpp"

PathWorker::PathWorker(QObject *parent)
    : QObject(parent)
{
    qDebug() << "Worker created in thread:" << QThread::currentThreadId();
}

PathWorker::~PathWorker(){
    qDebug() << "Worker destroyed in thread:" << QThread::currentThreadId();
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
    MapHelper::add_result(results, bfs_id.toStdString(),duration.count(),
                          data.first, bfs.get_travelled_nodes(), data.second);
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
    MapHelper::add_result(results, a_star_id.toStdString(),duration.count(),
                          data.first, as.get_travelled_nodes(), data.second);
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
    MapHelper::add_result(results, rrt_star_id.toStdString(), duration.count(),
                          data.first, rrt.get_travelled_nodes(), data.second);
}

// Compute path(s)
void PathWorker::compute_path(QString algo_name, Graph g, int max_iters){
    results.clear();
    QString err_msg;
    auto time_converted = TimeHelper::convert_from_ms(compute_timeout);
    int algos_finished = 0;
    emit algo_progress(algos_finished);
    if(algo_name == bfs_id || algo_name == all_id){
        this->run_bfs(g);
        if(timeout_occurred){
            err_msg += QString("   - BFS Computation exceeded %1 %2\n").arg(time_converted.first).arg(time_converted.second.c_str());
            timeout_occurred = false;
        }
        algos_finished++;
        emit algo_progress(algos_finished);
    }
    if(algo_name ==  a_star_id || algo_name == all_id){
        this->run_a_star(g);
        if(timeout_occurred){
            err_msg += QString("   - A* Computation exceeded %1 %2\n").arg(time_converted.first).arg(time_converted.second.c_str());
            timeout_occurred = false;
        }
        algos_finished++;
        emit algo_progress(algos_finished);
    }
    if(algo_name == rrt_star_id || algo_name == all_id){
        this->run_rrt_star(g, max_iters);
        if(timeout_occurred){
            err_msg += QString("   - RRT* Computation exceeded %1 %2\n").arg(time_converted.first).arg(time_converted.second.c_str());
            timeout_occurred = false;
        }
        algos_finished++;
        emit algo_progress(algos_finished);
    }

    if(!err_msg.isEmpty()) emit compute_error(results, err_msg);
    else emit compute_finished(results);
}
