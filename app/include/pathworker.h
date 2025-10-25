#ifndef PATHWORKER_H
#define PATHWORKER_H

#include <vector>

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QString>

#include "map_data.hpp"
#include "structs.hpp"
#include "map_helper.hpp"
#include "time_helper.hpp"

#include "app_constants.h"

class PathWorker : public QObject
{
    Q_OBJECT
public:
    explicit PathWorker(QObject *parent = nullptr);
    ~PathWorker();
    void send_timeout_error(QString& message);

public slots:
    void compute_path(QString algo_name, Graph g, SampleCountByAlgo samples, int neighbor_count, int step_size);

signals:
    void algo_progress(int completed);
    void compute_finished(vector<AlgoResult> results);
    void compute_error(vector<AlgoResult> results, const QString& message);

private:
    void run_bfs(Graph g);
    void run_a_star(Graph g);
    void run_rrt_star(Graph g, int sample_count, int step_size);
    void run_prm(Graph g, int sample_count, int neighbor_cout);

    int compute_timeout = AlgoConstants::DEFAULT_COMPUTE_TIMEOUT;
    bool timeout_occurred = false;

protected:
    vector<AlgoResult> results;
};

#endif // PATHWORKER_H
