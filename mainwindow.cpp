#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "bfs.hpp"
#include "a_star.hpp"
#include "rrt_star.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Path Planning Visualization");

    // Initialize elements of settings
    this->initialize_window();
}

MainWindow::~MainWindow(){
    delete ui;
}

void MainWindow::initialize_window(){
    QStringList algos_lst = {"BFS", "A*", "RRT*", "All"};
    ui->cb_bx_algos->addItems(algos_lst);

    inflate_size = ui->sp_bx_inflate->value();
    max_iters = ui->sp_bx_iterations->value();

    // Initialize map view
    scene = new QGraphicsScene(this);
    ui->view_map->setScene(scene);
    image = new QImage(ui->view_map->width(), ui->view_map->height(), QImage::Format_RGB666);
    image->fill(Qt::white);
    this->scene->addPixmap(QPixmap::fromImage(*image));
}

void MainWindow::update_pixmap(Map map, QImage *image){
    image->fill(empty_color);
    for(int row = 0; row < map.px_height; row++){
        for(int col = 0; col < map.px_width; col++){
            int cell_val = map.boundaries[row][col];
            if(cell_val >= 0){
                int* rgb = MapData::IDX2COLOR(cell_val);
                QColor color = QColor::fromRgb(rgb[0], rgb[1], rgb[2]);
                if(color.isValid()) image->setPixelColor(col, row, color);
                else image->setPixelColor(col, row, empty_color);
            }
        }
    }
}

void MainWindow::update_map(Map map){
    image = new QImage(map.px_width, map.px_height, QImage::Format_RGB666);
    this->update_pixmap(map, image);
    QPixmap px_map = QPixmap::fromImage(*image).scaled(ui->view_map->width(),
                                                        ui->view_map->height(),
                                                        Qt::KeepAspectRatio);
    this->scene->clear();
    this->scene->addPixmap(px_map);
}

void MainWindow::update_path(Map map, cell start, cell goal){
    Map shown_map;
    if(results.empty()) return;
    else if(results.size() == 1){
        if(debug) shown_map = MapData::debug_map(map, results[0].path, results[0].travelled, start, goal);
        else shown_map = MapData::add_path_to_map(map, results[0].path);
        this->update_map(shown_map);
    }
    else{
        shown_map = map;
        for(auto r: results){
            shown_map = MapData::add_path_to_map(shown_map, r.path);
        }
        this->update_map(shown_map);
    }
}

void MainWindow::on_btn_upload_map_clicked(){
    auto filename = QFileDialog::getOpenFileName(this, tr("Import Map YAML"), tr(""));
    if(filename.endsWith(".yaml")) {
        Map new_map = MapData::get_map(filename.toStdString());
        this->update_map(new_map);
        map = new_map;
    }
    else if(filename != "") QMessageBox::critical(this, "Import Error", filename + " is not valid. Make sure to provide a YAML file.");
}

void MainWindow::on_sp_bx_inflate_valueChanged(int val){
    try{
        inflate_size = val;
        map.boundaries = MapData::remove_boundary_inflation(map);
        map.boundaries = MapData::inflate_boundaries(map, inflate_size);
        ui->txt_results->setText(QString("Map obstacles inflated by %1.").arg(inflate_size));
        this->update_map(map);
        algo_run = false;
    }
    catch(std::bad_array_new_length b){
        std::cout << b.what() << std::endl;
    }
}

void MainWindow::on_btn_start_pos_clicked(){

}

void MainWindow::on_btn_goal_pos_clicked(){

}

void MainWindow::on_cb_bx_algos_currentTextChanged(const QString &name){
    algo_name = name;
    if(name == "All"){
        ui->ch_bx_debug->setChecked(false);
        ui->ch_bx_debug->setCheckable(false);
        ui->ch_bx_debug->hide();
    }
    else{
        ui->ch_bx_debug->setCheckable(true);
        ui->ch_bx_debug->show();
    }
}

void MainWindow::on_ch_bx_debug_toggled(bool checked){
    debug = checked;
    if(algo_run){
        this->update_results_view();
        this->update_path(map, graph.root, graph.end);
    }
}

void MainWindow::run_bfs(Graph g){
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
    if(debug) ar.travelled = bfs.get_travelled_nodes();
    results.push_back(ar);
}

void MainWindow::run_a_star(Graph g){
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
    if(debug) ar.travelled = as.get_travelled_nodes();
    results.push_back(ar);
}

void MainWindow::run_rrt_star(Graph g){
    auto rrt = RRTStar(g, map.px_width, map.px_height, max_iters);
    auto start_time = high_resolution_clock::now();
    rrt.solve(g.root, g.end);
    auto end_time = high_resolution_clock::now();
    auto data = rrt.reconstruct_path(g.root, g.end);
    AlgoResult ar;
    ar.type = "RRT*";
    ar.duration = duration_cast<milliseconds>(end_time- start_time).count();
    ar.path = data.first;
    ar.dist = data.second;
    if(debug) ar.travelled = rrt.get_travelled_nodes();
    results.push_back(ar);
}

void MainWindow::on_btn_run_algo_clicked(){
    graph = MapData::get_graph_from_map(map);
    QStringList start_pos_lst = ui->line_start_pos->text().split(",");
    start_pos = {start_pos_lst.value(0).toInt(), start_pos_lst.value(1).toInt()};
    QStringList goal_pos_lst = ui->line_goal_pos->text().split(",");
    goal_pos = {goal_pos_lst.value(0).toInt(), goal_pos_lst.value(1).toInt()};
    //qDebug() << start_pos;
    //qDebug() << goal_pos;

    ui->sp_bx_inflate->hide();
    if(!graph.is_node_valid(start_pos) || !graph.is_node_valid(goal_pos)){
        string err_msg = "Error:\n";
        if(!graph.is_node_valid(start_pos)) err_msg += "Start position is not valid\n";
        if(!graph.is_node_valid(goal_pos)) err_msg += "Goal position is not valid\n";
        err_msg += "Make sure the selected position\nis clear of obstacles and in\nthis format: \"int,int\"\n";
        ui->txt_results->setText(QString::fromStdString(err_msg));
    }
    else{
        graph.root = start_pos;
        graph.end = goal_pos;

        this->clear_results();
        if(algo_name == "BFS") this->run_bfs(graph);
        else if(algo_name ==  "A*") this->run_a_star(graph);
        else if(algo_name == "RRT*") this->run_rrt_star(graph);
        else if(algo_name =="All"){
            this->run_bfs(graph);
            this->run_a_star(graph);
            this->run_rrt_star(graph);
        }
        this->update_results_view();
        this->update_path(map, graph.root, graph.end);
        algo_run = true;
    }
    ui->sp_bx_inflate->show();
}

void MainWindow::clear_results(){
    results.clear();
    ui->txt_results->setText(QString("Data cleared. Running algoritm(s)..."));
    //qDebug() << "Data cleared. Running algoritm(s)...";
}

void MainWindow::update_results_view(){
    QString data = "";
    for(auto r : results){
        if(results.size()>1) data += QString("Algorithm: %1\n").arg(r.type);
        data += QString("Duration: %1 ms\n").arg(r.duration);
        data += QString("Distance: %1\n").arg(r.dist);
        if(debug){
            data += "Nodes: [ ";
            for(int i = 0; i<r.path.size(); i++){
                if(i == r.path.size()-1) data += QString("  %1,%2 ]\n").arg(r.path[i].first).arg(r.path[i].second);
                else data += QString("  %1,%2 ").arg(r.path[i].first).arg(r.path[i].second);
            }
        }
        data += "\n";
    }
   ui->txt_results->setText(data);
}
