#ifndef PATHWORKER_H
#define PATHWORKER_H

#include <vector>

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QString>

#include "map_data.hpp"

#include "map_helper.hpp"
#include "time_helper.hpp"

class PathWorker : public QObject
{
    Q_OBJECT
public:
    explicit PathWorker(QObject *parent = nullptr);
    ~PathWorker();
    void send_timeout_error(QString& message);

public slots:
    void compute_path(QString algo_name, Graph g, int max_iters);

signals:
    void algo_progress(int completed);
    void compute_finished(vector<AlgoResult> results);
    void compute_error(vector<AlgoResult> results, const QString& message);

private:
    void add_result(string algo_type, int duration, vector<cell> path, float dist, vector<cell> travelled);
    void run_bfs(Graph g);
    void run_a_star(Graph g);
    void run_rrt_star(Graph g, int max_iters);

    QString bfs_id = "BFS";
    QString a_star_id = "A*";
    QString rrt_star_id = "RRT*";
    QString all_id = "All";
    int compute_timeout = 600000;  // in milliseconds (10 minutes)
    bool timeout_occurred = false;

protected:
    vector<AlgoResult> results;
};

#endif // PATHWORKER_H
