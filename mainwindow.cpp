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

    // Force window to fixed dimensions
    //setFixedSize(870, 605);
    setFixedSize(this->geometry().width(),this->geometry().height());

    // Initialize elements of settings
    this->initialize_window();
}

MainWindow::~MainWindow(){
    delete ui;
}

void MainWindow::initialize_window(){
    // Initialize combobox for algorithms
    QStringList algos_lst = {"BFS", "A*", "RRT*", "All"};
    ui->cb_bx_algos->addItems(algos_lst);

    // Set max_iter to the current value
    max_iters = ui->sp_bx_iterations->value();

    // Initialize map view
    scene = new QGraphicsScene(this);
    ui->view_map->setScene(scene);
    image = new QImage(ui->view_map->width(), ui->view_map->height(), QImage::Format_RGB666);
    image->fill(Qt::white);
    this->scene->addPixmap(QPixmap::fromImage(*image));

    // Initialize color indexes for paths
    vector<array<int,3>> colors = {{1,1,1}, {0,0,0},{255,255,255},{128,0,128},{173,216,230},{255,0,0}, {102,178,255}, {230,230,0}, {0,179,60}, {255,166,77}, {0,128,128}, {51,0,153}};
    int path_idx = -2;
    for(auto color: colors){
        color_idxs.push_back(ColorIdx{path_idx, color});
        //qDebug() << path_idx << " <=> " << color;
        path_idx++;
    }
}

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
        else shown_map = MapData::add_path_to_map(map, results[0].path, start, goal);
        MapData::inflate_point(shown_map, start, pt_size);
        MapData::inflate_point(shown_map, goal, pt_size);
        this->update_map(shown_map);
    }
    else{
        shown_map = map;
        int path_idx = 3;
        for(auto r: results){
            shown_map = MapData::add_path_to_map_with_value(shown_map, path_idx, r.path, graph.root, graph.end);
            for(int i = 0; i < color_idxs.size(); i++){
                if(color_idxs[i].idx == path_idx) color_idxs[i].name = r.type;
            }
            path_idx++;
        }
        MapData::inflate_point(shown_map, start, pt_size);
        MapData::inflate_point(shown_map, goal, pt_size);
        this->update_map(shown_map);
    }
}

void MainWindow::on_btn_upload_map_clicked(){
    auto filename = QFileDialog::getOpenFileName(this, tr("Import Map YAML"), tr(""));
    if(filename.endsWith(".yaml")) {
        Map new_map = MapData::get_map(filename.toStdString());
        this->update_map(new_map);
        map = new_map;
        map_uploaded = true;
        ui->sp_bx_inflate->setValue(ui->sp_bx_inflate->minimum());
        this->clear_results();
        this->clear_results();
    }
    else if(filename != "") QMessageBox::critical(this, "Import Error", filename + " is not valid. Make sure to provide a YAML file.");
}

void MainWindow::on_btn_obstacles_clicked(){
    qDebug() << "TODO";
}

void MainWindow::on_sp_bx_inflate_valueChanged(int inflate_size){
    try{
        map.boundaries = MapData::remove_boundary_inflation(map);
        map.boundaries = MapData::inflate_boundaries(map, inflate_size);
        ui->txt_results->setText(QString("Map obstacles inflated by %1.").arg(inflate_size));
        this->update_map(map);
        path_computed = false;
    }
    catch(std::bad_array_new_length b){
        std::cout << b.what() << std::endl;
    }
}

void MainWindow::on_btn_start_pos_clicked(){
    qDebug() << "TODO";
}

void MainWindow::on_btn_goal_pos_clicked(){
    qDebug() << "TODO";
}

void MainWindow::on_cb_bx_algos_currentTextChanged(const QString &name){
    algo_name = name;

    // Update debug checkbox
    if(name == "All"){
        ui->ch_bx_debug->setChecked(false);
        ui->ch_bx_debug->setCheckable(false);
        ui->ch_bx_debug->hide();
    }
    else{
        ui->ch_bx_debug->setCheckable(true);
        ui->ch_bx_debug->show();
    }

    // Update max iterations spinbox
    if(name == "RRT*" || name == "All"){
        ui->lbl_iterations->show();
        ui->sp_bx_iterations->show();
    }else{
        ui->lbl_iterations->hide();
        ui->sp_bx_iterations->hide();
    }

    // Reset results text and map when selected algorithm changed
    if(path_computed){
        this->update_map(map);
        this->clear_results();
        ui->txt_results->setText("Data cleared. Hit \"Run\" to get results.");
    }
}

void MainWindow::on_ch_bx_debug_toggled(bool checked){
    debug = checked;
    if(path_computed){
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
    ar.travelled = bfs.get_travelled_nodes();
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
    ar.travelled = as.get_travelled_nodes();
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
    ar.travelled = rrt.get_travelled_nodes();
    results.push_back(ar);
}

cell MainWindow::get_positon(string pos_str){
    cell pos;
    try{
        string val;
        char delim = ',';
        stringstream pos_ss(pos_str);
        getline(pos_ss, val, delim);
        pos.first = std::stoi(val);
        getline(pos_ss, val, delim);
        pos.second = std::stoi(val);
    } catch(std::invalid_argument e){
        pos = {-1,-1};
    }
    return pos;
}

void MainWindow::on_btn_run_algo_clicked(){
    // Convert map into a graph
    graph = MapData::get_graph_from_map(map);

    // Get start and goal position
    start_pos = get_positon(ui->line_start_pos->text().toStdString());
    goal_pos = get_positon(ui->line_goal_pos->text().toStdString());

    // Run algorithm(s)
    ui->sp_bx_inflate->hide();
    if(!map_uploaded) ui->txt_results->setText("Error:\n  - Map has not been uploaded yet.");
    else if(!graph.is_node_valid(start_pos) || !graph.is_node_valid(goal_pos)){
        string err_msg = "Error:\n";
        if(!graph.is_node_valid(start_pos)) err_msg += "  - Start position is not valid\n";
        if(!graph.is_node_valid(goal_pos)) err_msg += "  - Goal position is not valid\n";
        err_msg += "\nMake sure the position text field is not empty, is clear of obstacles, and in this format: \"int,int\"\n";
        ui->txt_results->setText(QString::fromStdString(err_msg));
    }
    else{
        graph.root = start_pos;
        graph.end = goal_pos;
        max_iters = ui->sp_bx_iterations->value();
        this->clear_results();
        path_computed = false;
        ui->txt_results->setText("Data cleared. Running algoritm(s)...");
        if(algo_name == "BFS") this->run_bfs(graph);
        else if(algo_name ==  "A*") this->run_a_star(graph);
        else if(algo_name == "RRT*") this->run_rrt_star(graph);
        else if(algo_name =="All"){
            this->run_bfs(graph);
            this->run_a_star(graph);
            this->run_rrt_star(graph);
        }
        this->update_path(map, graph.root, graph.end);
        this->update_results_view();
        path_computed = true;
    }
    ui->sp_bx_inflate->show();
}

void MainWindow::clear_results(){
    results.clear();
}

void MainWindow::update_results_view(){
    QString data = "";
    for(auto r : results){
        QString sub_data = "";
        if(results.size()>1) sub_data += QString("Algorithm: %1 <br>").arg(r.type);
        sub_data += QString("Duration: %1 ms <br>").arg(r.duration);
        sub_data += QString("Distance: %1 <br>").arg(r.dist);
        if(debug){
            sub_data += "Path Nodes: [ ";
            for(int i = 0; i<r.path.size(); i++){
                if(i == r.path.size()-1) sub_data += QString("  %1,%2 ] <br>").arg(r.path[i].first).arg(r.path[i].second);
                else sub_data += QString("  %1,%2 ").arg(r.path[i].first).arg(r.path[i].second);
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
