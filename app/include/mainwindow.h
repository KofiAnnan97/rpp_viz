#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <string>

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
#include <QCursor>

#include "map_data.hpp"
#include "map_helper.hpp"

#include "pathworker.h"
#include "drawingpanel.h"

using namespace std::chrono;
using namespace std;

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
    void initialize_window();
    void clear_results();
    void update_results_view();

private:
    QCursor set_erase_cursor(int erase_size);
    void set_settings_enabled(bool is_enabled);
    void set_position_button(QPushButton *obj, bool is_enabled);
    bool eventFilter(QObject *object, QEvent *event);

    // UI Variables
    Ui::MainWindow *ui;
    QColor empty_color = Qt::black;
    bool start_pos_click = false;
    QString last_start_pos_str;
    bool goal_pos_click = false;
    QString last_goal_pos_str;
    bool draw_click = false;
    bool erase_click = false;
    QPoint mouse_pos;
    int num_of_algos;

    // For multi-threading
    QThread *worker_thread;
    PathWorker *p_worker;

    // State Variables
    DrawingPanel *draw_panel;
    Graph graph;
    bool path_computed = false;
    QString algo_name;
    int max_iters = 10000;
    cell start_pos = {-1,-1};
    cell goal_pos = {-1,-1};
    vector<AlgoResult> results;

    // Constants
    const int pt_size = 5;
    const int COLOR_PATH_IDX = 3;
    const QString draw_btn_icon = ":/icons/pencil.svg";
    const QString erase_btn_icon = ":/icons/eraser.svg";
    const QString draw_cursor = ":/icons/cursor_pencil.svg";
    const QString erase_cursor_small = ":/icons/cursor_eraser_small.svg";
    const QString erase_cursor = ":/icons/cursor_eraser.svg";
    const QString bfs_id = "BFS";
    const QString a_star_id = "A*";
    const QString rrt_star_id = "RRT*";
    const QString all_id = "All";

private slots:
    void on_btn_upload_map_clicked();
    void on_btn_draw_clicked();
    void on_btn_erase_clicked();
    void on_sp_bx_draw_size_valueChanged(int draw_size);
    void on_sp_bx_erase_size_valueChanged(int erase_size);
    void on_ch_bx_match_inflate_toggled(bool checked);
    void on_sp_bx_inflate_valueChanged(int val);
    void on_line_start_pos_editingFinished();
    void on_line_goal_pos_editingFinished();
    void on_btn_start_pos_clicked();
    void on_btn_goal_pos_clicked();
    void on_cb_bx_algos_currentTextChanged(const QString &name);
    void on_ch_bx_debug_toggled(bool checked);
    void on_btn_run_algo_clicked();
    void handle_thread_finished();
    void handle_algo_progress(int val);
    void handle_compute_path_finished(vector<AlgoResult> results);
    void handle_compute_path_error(vector<AlgoResult> results, const QString& message);
};
#endif // MAINWINDOW_H
