#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <string>
#include <iomanip>
#include <sstream>

#include <QMainWindow>
#include <QString>
#include <QFileDialog>
#include <QDebug>
#include <QStringList>
#include <QMessageBox>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPixmap>
#include <QImage>
#include <QColor>
#include <QRgb>
#include <QPoint>
#include <QPointF>
#include <QMouseEvent>
#include <QThread>

#include "map_data.hpp"

using namespace std::chrono;
using namespace std;

typedef std::chrono::_V2::system_clock::time_point c_time_point;

struct ColorIdx{
    int idx;
    array<int,3> rgb_vals;
    string name;
};

struct AlgoResult{
    string type;
    int duration;
    vector<cell> path;
    vector<cell> travelled;
    float dist;
};

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void update_map(Map map);
    void update_path(Map map, cell start, cell goal);
    void clear_results();
    void update_results_view();

private:
    void update_pixmap(Map map, QImage* image);
    void initialize_window();
    cell get_positon(string pos_str);
    void run_bfs(Graph g);
    void run_a_star(Graph g);
    void run_rrt_star(Graph g);
    void set_settings_enabled(bool is_enabled);
    bool eventFilter(QObject *object, QEvent *event);

    // UI Variables
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QImage *image;
    QPixmap px_map;
    QColor empty_color = Qt::black;
    bool start_pos_click = false;
    bool goal_pos_click = false;
    bool alter_map_click = false;
    QPoint mouse_pos;

    // State Variables
    Map map;
    Graph graph;
    bool debug = false;
    bool path_computed = false;
    bool map_uploaded = false;
    QString algo_name;
    int max_iters = 10000;
    const int pt_size = 5;
    cell start_pos = {-1,-1};
    cell goal_pos = {-1,-1};
    vector<AlgoResult> results;
    vector<ColorIdx> color_idxs;

private slots:
    void on_btn_upload_map_clicked();
    void on_btn_obstacles_clicked();
    void on_sp_bx_inflate_valueChanged(int val);
    void on_btn_start_pos_clicked();
    void on_btn_goal_pos_clicked();
    void on_cb_bx_algos_currentTextChanged(const QString &name);
    void on_ch_bx_debug_toggled(bool checked);
    void on_btn_run_algo_clicked();

    //void mouseMoveEvent(QMouseEvent *event) override;

};
#endif // MAINWINDOW_H
