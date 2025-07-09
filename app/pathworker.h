#ifndef PATHWORKER_H
#define PATHWORKER_H

#include <vector>

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QString>

#include "map_data.hpp"
#include "bfs.hpp"
#include "a_star.hpp"
#include "rrt_star.hpp"

using namespace std::chrono;

typedef std::chrono::_V2::system_clock::time_point c_time_point;

struct AlgoResult{
    string type;
    int duration;
    vector<cell> path;
    vector<cell> travelled;
    float dist;
};

class PathWorker : public QObject
{
    Q_OBJECT
public:
    explicit PathWorker(QObject *parent = nullptr);
    ~PathWorker();

public slots:
    void compute_path(QString algo_name, Graph g, int width, int height, int max_iters);

signals:
    void compute_finished(vector<AlgoResult> results);
    void compute_error(const QString& message);

private:
    void run_bfs(Graph g);
    void run_a_star(Graph g);
    void run_rrt_star(Graph g, int px_width, int px_height, int max_iters);

    bool m_stop_requested;

protected:
    vector<AlgoResult> results;
};

#endif // PATHWORKER_H
