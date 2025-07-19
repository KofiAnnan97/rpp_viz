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

using namespace std::chrono;
using namespace std;

struct ColorIdx{
    int idx;
    array<int,3> rgb_vals;
    string name;
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
    void initialize_window();
    void update_map(Map map);
    void show_path(Map map, cell start, cell goal);
    void clear_results();
    void update_results_view();

private:
    void set_settings_enabled(bool is_enabled);
    void set_position_button(QPushButton *obj, bool is_enabled);
    void update_pixmap(Map map, QImage* image);
    void add_point_to_display(QString last_pos_str, QString pos_str);
    bool eventFilter(QObject *object, QEvent *event);

    // UI Variables
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QImage *image;
    QPixmap px_map;
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
    Map obstacle_map, display_map;
    Graph graph;
    bool debug = false;
    bool path_computed = false;
    bool map_uploaded = false;
    QString algo_name;
    int max_iters = 10000;
    cell start_pos = {-1,-1};
    cell goal_pos = {-1,-1};
    vector<AlgoResult> results;
    vector<ColorIdx> color_idxs;

    // Constants
    const int pt_size = 5;
    const int COLOR_PATH_IDX = 3;
    const QString draw_btn_icon = "../../app/icons/pencil.svg";
    const QString erase_btn_icon = "../../app/icons/eraser.svg";
    const QString draw_cursor = "../../app/icons/cursor_pencil.svg";
    const QString erase_cursor = "../../app/icons/cursor_eraser.svg";

private slots:
    void on_btn_upload_map_clicked();
    void on_btn_draw_clicked();
    void on_btn_erase_clicked();
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
