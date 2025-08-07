#include <sstream>

#include "mainwindow.h"
#include "../ui_mainwindow.h"
#include "pathworker.h"
#include "time_helper.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Path Planning Visualization");

    // Force window to fixed dimensions
    setFixedSize(this->geometry().width(),this->geometry().height());

    // Initialize elements of settings
    this->initialize_window();
}

MainWindow::~MainWindow(){
    delete ui;
}

void MainWindow::initialize_window(){
    // Initialize combobox for algorithms
    QStringList algos_lst = {bfs_id, a_star_id, rrt_star_id, all_id};
    ui->cb_bx_algos->addItems(algos_lst);
    num_of_algos = algos_lst.size()-1;

    // Set max_iter to the current value
    max_iters = ui->sp_bx_iterations->value();

    // Initialize map display
    scene = new QGraphicsScene(this);
    ui->view_map->setScene(scene);
    image = new QImage(ui->view_map->width(), ui->view_map->height(), QImage::Format_RGB666);
    image->fill(Qt::white);
    this->scene->addPixmap(QPixmap::fromImage(*image));

    // Allow for mouse events on map display
    ui->view_map->setMouseTracking(true);
    ui->view_map->installEventFilter(this);
    scene->installEventFilter(this);

    // Initialize color indexes for paths (only supports 7 paths)
    vector<array<int,3>> colors = {{1,1,1}, {0,0,0},{255,255,255},{128,0,128},{173,216,230},{255,0,0}, {102,178,255}, {0,179,60}, {230,230,0}, {255,166,77}, {0,128,128}, {51,0,153}};
    int path_idx = -2;
    for(auto color: colors){
        color_idxs.push_back(ColorIdx{path_idx, color});
        //qDebug() << path_idx << " <=> " << color;
        path_idx++;
    }

    // Set up draw and erase buttons
    ui->btn_draw->setIcon(QIcon(draw_btn_icon));
    ui->btn_erase->setIcon(QIcon(erase_btn_icon));
    ui->ch_bx_match_inflate->setChecked(true);
    ui->btn_obstacles->hide();
}

// HELPER FUNCTIONS

void MainWindow::set_position_button(QPushButton *obj, bool is_enabled){
    // Change text of button
    if(is_enabled){
        obj->setText("x");
        ui->view_map->viewport()->setCursor(Qt::CrossCursor);
    }
    else{
        obj->setText("o");
        ui->view_map->viewport()->setCursor(Qt::ArrowCursor);
    }

    // Determine if button active
    if(obj == ui->btn_start_pos && start_pos_click != is_enabled)
        start_pos_click = is_enabled;
    else if(obj == ui->btn_goal_pos && goal_pos_click != is_enabled)
        goal_pos_click = is_enabled;
}

void MainWindow::set_settings_enabled(bool is_enabled){
    ui->btn_upload_map->setEnabled(is_enabled);
    ui->btn_draw->setEnabled(is_enabled);
    ui->btn_erase->setEnabled(is_enabled);
    ui->sp_bx_draw_size->setEnabled(is_enabled);
    ui->sp_bx_erase_size->setEnabled(is_enabled);
    ui->ch_bx_match_inflate->setEnabled(is_enabled);
    ui->sp_bx_inflate->setEnabled(is_enabled);
    ui->line_start_pos->setEnabled(is_enabled);
    ui->line_goal_pos->setEnabled(is_enabled);
    ui->btn_start_pos->setEnabled(is_enabled);
    ui->btn_goal_pos->setEnabled(is_enabled);
    ui->cb_bx_algos->setEnabled(is_enabled);
    ui->sp_bx_iterations->setEnabled(is_enabled);
    ui->btn_run_algo->setEnabled(is_enabled);
}

// MAP DISPLAY FUNCTIONS

void MainWindow::update_pixmap(Map map, QImage *image){
    image->fill(empty_color);
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            int cell_val = map.boundaries[row][col];
            int *rgb;
            for(int i = 0; i < color_idxs.size(); i++){
                if(color_idxs[i].idx == cell_val) {
                    rgb = new int[3];
                    rgb[0] = color_idxs[i].rgb_vals[0];
                    rgb[1] = color_idxs[i].rgb_vals[1];
                    rgb[2] = color_idxs[i].rgb_vals[2];
                    break;
                }
                else if(color_idxs[i].idx != cell_val && i == color_idxs.size()-1){
                    rgb = new int[3];
                    rgb[0] = 128;
                    rgb[1] = 128;
                    rgb[2] = 128;
                }
            }
            QColor color = QColor::fromRgb(rgb[0], rgb[1], rgb[2]);
            if(color.isValid()) image->setPixelColor(col, row, color);
            else image->setPixelColor(col, row, empty_color);
        }
    }
}

void MainWindow::update_map(Map map){
    image = new QImage(map.px_width, map.px_height, QImage::Format_RGB666);
    this->update_pixmap(map, image);
    px_map = QPixmap::fromImage(*image).scaled(ui->view_map->width(),
                                                        ui->view_map->height(),
                                                        Qt::KeepAspectRatio);
    this->scene->clear();
    this->scene->addPixmap(px_map);
}

void MainWindow::show_path(Map map, cell start, cell goal){
    Map path_map = MapData::copy_map(display_map);
    if(results.empty()) return;
    else if(results.size() == 1){
        if(debug) path_map = MapData::debug_map(path_map, results[0].path, results[0].travelled, start, goal);
        else path_map = MapData::add_path_to_map(path_map, results[0].path, start, goal);
    }
    else{
        path_map = map;
        int path_idx = COLOR_PATH_IDX;
        for(auto r: results){
            path_map = MapData::add_path_to_map_with_value(path_map, path_idx, r.path, graph.root, graph.end);
            for(int i = 0; i < color_idxs.size(); i++){
                if(color_idxs[i].idx == path_idx) color_idxs[i].name = r.type;
            }
            path_idx++;
        }
    }
    path_map.boundaries[start.second][start.first] = MapData::NAV_POINT_INT;
    MapData::inflate_point(path_map, start, pt_size);
    path_map.boundaries[goal.second][goal.first] = MapData::NAV_POINT_INT;
    MapData::inflate_point(path_map, goal, pt_size);
    this->update_map(path_map);
    delete path_map.boundaries;
}

void MainWindow::add_point_to_display(QString last_pos_str, QString pos_str){
    if(map_uploaded){
        // Remove old point if applicable
        if(!last_pos_str.isEmpty() && last_pos_str != pos_str){
            auto last_pos = MapHelper::get_positon(last_pos_str.toStdString());
            if(last_pos.first >= 0 && last_pos.first < display_map.px_width && last_pos.second >= 0 && last_pos.second < display_map.px_height){
                display_map.boundaries[last_pos.second][last_pos.first] = MapData::OPEN_SPACE_INT;
                MapData::inflate_point(display_map, last_pos, MapData::POINT_SIZE);
            }
        }

        // Add new point
        if(!pos_str.isEmpty()){
            auto pos = MapHelper::get_positon(pos_str.toStdString());
            if(pos.first >= 0 && pos.first < display_map.px_width && pos.second >= 0 && pos.second < display_map.px_height){
                display_map.boundaries[pos.second][pos.first] = MapData::NAV_POINT_INT;
                MapData::inflate_point(display_map, pos, MapData::POINT_SIZE);
                this->update_map(display_map);
            }
        }
    }
}

bool MainWindow::eventFilter(QObject *object, QEvent *event){
    if(!map_uploaded && event->type() == QEvent::MouseButtonPress) {
        ui->txt_results->setText("Please upload map first.");
        if(start_pos_click) this->set_position_button(ui->btn_start_pos, false);
        if(goal_pos_click) this->set_position_button(ui->btn_goal_pos, false);
    }
    else if(object == ui->view_map && event->type() == QEvent::MouseButtonPress){
        QMouseEvent *mouse_event = static_cast<QMouseEvent*>(event);
        mouse_pos = mouse_event->pos();
        QPointF scene_pos = ui->view_map->mapToScene(mouse_pos);
        int scaled_x = std::round(scene_pos.x()*((float)obstacle_map.px_width/px_map.width()));
        int scaled_y = std::round(scene_pos.y()*((float)obstacle_map.px_height/px_map.height()));
        //qDebug() << scaled_x << "," << scaled_y;

        if(mouse_event->button() == Qt::LeftButton){
            // WORK IN PROGRESS: Set to update map when mouse is dragged after
            //                   initial press until button release
            //qDebug() << "pressed";
            /*while(true){
                if(event->type() == QEvent::GraphicsSceneMouseRelease) break;  //MouseButtonRelease){
                qDebug() << "a";
            }*/

            // WORKING CODE
            if(draw_click || erase_click){
                if(erase_click){
                    obstacle_map.boundaries[scaled_y][scaled_x] = MapData::OPEN_SPACE_INT;
                    MapData::inflate_point(obstacle_map, {scaled_x, scaled_y}, ui->sp_bx_erase_size->value());
                }
                else if(draw_click){
                    obstacle_map.boundaries[scaled_y][scaled_x] = MapData::OBSTACLE_INT;
                    MapData::inflate_point(obstacle_map, {scaled_x, scaled_y}, ui->sp_bx_draw_size->value());
                }
                // Update display map with start and goal position
                display_map.boundaries = MapData::copy_boundaries(obstacle_map);
                auto start_pos_str = ui->line_start_pos->text();
                auto goal_pos_str = ui->line_goal_pos->text();
                if(!start_pos_str.isEmpty()) this->add_point_to_display(start_pos_str, start_pos_str);
                if(!goal_pos_str.isEmpty()) this->add_point_to_display(goal_pos_str, goal_pos_str);
                if(start_pos_str.isEmpty() && goal_pos_str.isEmpty()) this->update_map(display_map);
                return true;
            }
            else if(start_pos_click){
                ui->line_start_pos->setText(QString("%1,%2").arg(scaled_x).arg(scaled_y));
                this->set_position_button(ui->btn_start_pos, false);
                this->add_point_to_display(last_start_pos_str, ui->line_start_pos->text());
                last_start_pos_str = ui->line_start_pos->text();
                return true;
            }
            else if(goal_pos_click){
                ui->line_goal_pos->setText(QString("%1,%2").arg(scaled_x).arg(scaled_y));
                this->set_position_button(ui->btn_goal_pos, false);
                this->add_point_to_display(last_goal_pos_str, ui->line_goal_pos->text());
                last_goal_pos_str = ui->line_goal_pos->text();
                return true;
            }
        }
    }
    return false;
}

// MAP SETTINGS FUNCTIONS

void MainWindow::on_btn_upload_map_clicked(){
    auto filename = QFileDialog::getOpenFileName(this, tr("Import Map YAML"), tr(""));
    //QString filename = "../../resources/maps/example1.yaml";
    if(filename.endsWith(".yaml")) {
        Map new_map = MapData::get_map(filename.toStdString());
        this->update_map(new_map);
        obstacle_map = new_map;
        display_map = MapData::copy_map(new_map);
        map_uploaded = true;
        draw_click = false;
        erase_click = false;
        ui->view_map->viewport()->setCursor(Qt::ArrowCursor);
        ui->sp_bx_inflate->setValue(ui->sp_bx_inflate->minimum());
        this->clear_results();
        this->update_results_view();
    }
    else if(filename != "") QMessageBox::critical(this, "Import Error", filename + " is not valid. Make sure to provide a YAML file.");
}

void MainWindow::on_btn_draw_clicked(){
    // Reset start position, goal position, and erase buttons
    if(start_pos_click) this->set_position_button(ui->btn_start_pos, false);
    if(goal_pos_click) this->set_position_button(ui->btn_goal_pos, false);
    erase_click = false;

    // Set state of drawing obstacles on map
    draw_click = !draw_click;
    QCursor cursor;
    if(draw_click) cursor = QCursor(QPixmap(draw_cursor).scaled(20,20), 0, 20);
    else cursor = QCursor(Qt::ArrowCursor);
    ui->view_map->viewport()->setCursor(cursor);
}

void MainWindow::on_btn_erase_clicked(){
    // Reset start position, goal position, and erase buttons
    if(start_pos_click) this->set_position_button(ui->btn_start_pos, false);
    if(goal_pos_click) this->set_position_button(ui->btn_goal_pos, false);
    draw_click = false;

    // Set state of erasing obstacles on map
    erase_click = !erase_click;
    QCursor cursor;
    int e_size = ui->sp_bx_erase_size->value();
    if(erase_click) cursor = QCursor(QPixmap(erase_cursor).scaled(e_size,e_size), e_size/2, e_size/2);
    else cursor = QCursor(Qt::ArrowCursor);
    ui->view_map->viewport()->setCursor(cursor);
}

void MainWindow::on_sp_bx_draw_size_valueChanged(int erase_size){
    if(ui->sp_bx_inflate->value() != ui->sp_bx_draw_size->value())
        ui->ch_bx_match_inflate->setChecked(false);
}

void MainWindow::on_sp_bx_erase_size_valueChanged(int erase_size){
    if(erase_click){
        auto cursor = QCursor(QPixmap(erase_cursor).scaled(erase_size,erase_size), erase_size/2, erase_size/2);
        ui->view_map->viewport()->setCursor(cursor);    
    }

    if(ui->sp_bx_inflate->value() != ui->sp_bx_erase_size->value())
        ui->ch_bx_match_inflate->setChecked(false);
}

void MainWindow::on_ch_bx_match_inflate_toggled(bool checked){
    if(checked){
        ui->sp_bx_draw_size->setValue(ui->sp_bx_inflate->value());
        ui->sp_bx_erase_size->setValue(ui->sp_bx_inflate->value());
    }
}

void MainWindow::on_sp_bx_inflate_valueChanged(int inflate_size){
    // Update map with inflated obstacles if available
    if(map_uploaded){
        obstacle_map.boundaries = MapData::remove_boundary_inflation(obstacle_map);
        obstacle_map.boundaries = MapData::inflate_boundaries(obstacle_map, inflate_size);
        display_map.boundaries = MapData::copy_boundaries(obstacle_map);
        ui->txt_results->setText(QString("Map obstacles inflated by %1.").arg(inflate_size));

        QString start_pos_str = ui->line_start_pos->text();
        QString goal_pos_str = ui->line_goal_pos->text();
        if(!start_pos_str.isEmpty()) this->add_point_to_display(start_pos_str, start_pos_str);
        if(!goal_pos_str.isEmpty()) this->add_point_to_display(goal_pos_str, goal_pos_str);
        if(start_pos_str.isEmpty() && goal_pos_str.isEmpty()) this->update_map(obstacle_map);
        path_computed = false;
    }

    // Match draw and erase size to inflate size if box checked
    if(ui->ch_bx_match_inflate->isChecked()){
        ui->sp_bx_draw_size->setValue(inflate_size);
        ui->sp_bx_erase_size->setValue(inflate_size);
    }
}

void MainWindow::on_line_start_pos_editingFinished(){
    this->add_point_to_display(last_start_pos_str, ui->line_start_pos->text());
    last_start_pos_str = ui->line_start_pos->text();
}

void MainWindow::on_line_goal_pos_editingFinished(){
    this->add_point_to_display(last_goal_pos_str, ui->line_goal_pos->text());
    last_goal_pos_str = ui->line_goal_pos->text();
}

void MainWindow::on_btn_start_pos_clicked(){
    start_pos_click = !start_pos_click;
    if(start_pos_click) {
        this->set_position_button(ui->btn_goal_pos, false);
        this->set_position_button(ui->btn_start_pos, true);
        if(draw_click || erase_click){
            draw_click = false;
            erase_click = false;
        }
    }
    else this->set_position_button(ui->btn_start_pos, false);
    draw_click = false;
    erase_click = false;
}

void MainWindow::on_btn_goal_pos_clicked(){
    goal_pos_click = !goal_pos_click;
    if(goal_pos_click) {
        this->set_position_button(ui->btn_start_pos, false);
        this->set_position_button(ui->btn_goal_pos, true);
        if(draw_click || erase_click){
            draw_click = false;
            erase_click = false;
        }
    }
    else this->set_position_button(ui->btn_goal_pos, false);
    draw_click = false;
    erase_click = false;
}

// ALGORITHM SETTINGS FUNCTIONS

void MainWindow::on_cb_bx_algos_currentTextChanged(const QString &name){
    algo_name = name;

    // Update debug checkbox
    if(name == all_id){
        ui->ch_bx_debug->setChecked(false);
        ui->ch_bx_debug->setCheckable(false);
        ui->ch_bx_debug->hide();
    }
    else{
        ui->ch_bx_debug->setCheckable(true);
        ui->ch_bx_debug->show();
    }

    // Update max iterations spinbox
    if(name == rrt_star_id || name == all_id){
        ui->lbl_iterations->show();
        ui->sp_bx_iterations->show();
    }else{
        ui->lbl_iterations->hide();
        ui->sp_bx_iterations->hide();
    }

    // Reset results text and map when selected algorithm changed
    if(path_computed){
        this->update_map(display_map);
        this->clear_results();
        ui->txt_results->setText("Data cleared. Hit \"Run\" to get results.");
        path_computed = false;
    }
}

void MainWindow::on_ch_bx_debug_toggled(bool checked){
    debug = checked;
    if(path_computed){
        this->update_results_view();
        this->show_path(obstacle_map, graph.root, graph.end);
    }
}

void MainWindow::handle_compute_path_finished(vector<AlgoResult> c_results){
    results = c_results;
    this->show_path(obstacle_map, graph.root, graph.end);
    this->update_results_view();
    path_computed = true;
    this->set_settings_enabled(true);
}

void MainWindow::handle_compute_path_error(vector<AlgoResult> c_results, const QString& message){
    auto err_msg = QString("STATUS:\n%1").arg(message);
    QMessageBox::critical(this, "Computation Error", err_msg);
    this->handle_compute_path_finished(c_results);
}

void MainWindow::on_btn_run_algo_clicked(){
    // Get start and goal position
    start_pos = MapHelper::get_positon(ui->line_start_pos->text().toStdString());
    goal_pos = MapHelper::get_positon(ui->line_goal_pos->text().toStdString());

    // Convert map to graph
    if(map_uploaded) graph = MapData::get_graph_from_map(obstacle_map);

    // Run algorithm(s)
    if(!map_uploaded){
        this->set_settings_enabled(false);
        QMessageBox::critical(this, "Map Error ", "Map has not been uploaded yet. Please click the \"Upload Map\" button to retrieve a map.");
        this->set_settings_enabled(true);
    }
    else if(!graph.is_node_valid(start_pos) || !graph.is_node_valid(goal_pos)){
        string err_msg = "Error:\n";
        if(!graph.is_node_valid(start_pos)) err_msg += "  - Start position is not valid\n";
        if(!graph.is_node_valid(goal_pos)) err_msg += "  - Goal position is not valid\n";
        err_msg += "\nMake sure the position text field is not empty, is clear of obstacles, and in this format: \"int,int\"\n";
        ui->txt_results->setText(QString::fromStdString(err_msg));
        this->set_settings_enabled(true);
    }
    else{
        graph.root = start_pos;
        graph.end = goal_pos;

        max_iters = ui->sp_bx_iterations->value();
        if(path_computed){
            this->update_map(display_map);
            path_computed = false;
        }
        this->set_settings_enabled(false);


        // Create thread for running path computation
        worker_thread = new QThread;
        p_worker = new PathWorker();
        p_worker->moveToThread(worker_thread);
        connect(worker_thread, &QThread::started, p_worker, [this]{
            p_worker->compute_path(algo_name, graph, max_iters);
        });

        // Set signal for MainWindow functions
        connect(p_worker, &PathWorker::algo_progress, this, &MainWindow::handle_algo_progress);
        connect(p_worker, &PathWorker::compute_finished, this, &MainWindow::handle_compute_path_finished);
        connect(p_worker, &PathWorker::compute_error, this, &MainWindow::handle_compute_path_error);

        // Exit worker
        connect(p_worker, &PathWorker::compute_finished, worker_thread, &QThread::quit);
        connect(p_worker, &PathWorker::compute_error, worker_thread, &QThread::quit);

        // Thread cleanup
        connect(worker_thread, &QThread::finished, this, &MainWindow::handle_thread_finished);

        // Start worker thread
        worker_thread->start();
    }
}

// RESULTS DISPLAY FUNCTIONS

void MainWindow::clear_results(){
    results.clear();
}

void MainWindow::handle_algo_progress(int val){
    int max;
    if(ui->cb_bx_algos->currentText() != all_id) max = 1;
    else max = num_of_algos;
    ui->txt_results->setText(QString("Running algoritm(s)...\nCompleted: %1/%2").arg(val).arg(max));
}

void MainWindow::update_results_view(){
    QString data = "";
    for(auto r : results){
        QString sub_data = "";
        if(results.size()>1) sub_data += QString("Algorithm: %1 <br>").arg(r.type.c_str());
        auto duration_converted = TimeHelper::convert_from_ms(r.duration);
        sub_data += QString("Duration: %1 %2 <br>").arg(duration_converted.first).arg(duration_converted.second.c_str());
        sub_data += QString("Distance: %1 <br>").arg(r.dist);
        if(debug){
            sub_data += "Path Nodes: [ ";
            for(int i = 0; i<r.path.size(); i++){
                if(i == r.path.size()-1) sub_data += QString("  (%1,%2) ] <br> <br>").arg(r.path[i].first).arg(r.path[i].second);
                else sub_data += QString("  (%1,%2) ").arg(r.path[i].first).arg(r.path[i].second);
            }
        }
        if(results.size() == 1) data += sub_data;
        else {
            QColor text_color;
            for(auto ci: color_idxs){
                if(ci.name == r.type){
                    text_color = QColor(ci.rgb_vals[0], ci.rgb_vals[1], ci.rgb_vals[2]);
                    break;
                }
            }
            data += QString("<font color=\"%1\">%2<br></font>").arg(text_color.name()).arg(sub_data);
        }
    }
   ui->txt_results->setText(data);
}

// THREAD FUNCTIONS

void MainWindow::handle_thread_finished(){
    qDebug() << "Worker thread finished and cleaned up.";
    p_worker->deleteLater();
    //worker_thread->deleteLater();
    worker_thread->terminate();
    worker_thread->wait();
    p_worker = nullptr;
    worker_thread = nullptr;
}
