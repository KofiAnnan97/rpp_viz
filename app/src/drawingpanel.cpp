#include "drawingpanel.h"

DrawingPanel::DrawingPanel(QGraphicsView *parent){
    // Initialize map display
    scene = new QGraphicsScene(parent);
    scene->setSceneRect(0, 0, parent->width(), parent->height());
    parent->setScene(scene);
    image = new QImage(scene->width(), scene->height(), QImage::Format_RGB666);
    image->fill(Qt::white);
    scene->addPixmap(QPixmap::fromImage(*image));
    parent->setDragMode(QGraphicsView::NoDrag);
    scene->installEventFilter(this);

    // Initialize color indexes for paths (only supports 7 paths)
    vector<array<int,3>> colors = {{1,1,1}, {0,0,0},{255,255,255},{128,0,128},{173,216,230},{255,0,0}, {102,178,255}, {0,179,60}, {230,230,0}, {255,166,77}, {0,128,128}, {51,0,153}};
    int path_idx = -2;
    for(auto color: colors){
        color_idxs.push_back(ColorIdx{path_idx, color});
        //qDebug() << path_idx << " <=> " << color;
        path_idx++;
    }
}

// Setters
void DrawingPanel::set_pen_state(bool is_enabled){
    pen_enabled = is_enabled;
    if(pen_enabled) eraser_enabled = false;
}

void DrawingPanel::set_eraser_state(bool is_enabled){
    eraser_enabled = is_enabled;
    if(eraser_enabled) pen_enabled = false;
}

void DrawingPanel::set_map_state(bool map_state){
    map_uploaded = map_state;
}

void DrawingPanel::set_obstacle_map(Map map){
    obstacle_map = map;
}

void DrawingPanel::set_display_map(Map map){
    display_map = map;
}

void DrawingPanel::set_debug_state(bool debug_state){
    debug = debug_state;
}

// Getters
bool DrawingPanel::is_map_uploaded(){
    return map_uploaded;
}

bool DrawingPanel::debug_state(){
    return debug;
}

vector<ColorIdx> DrawingPanel::get_color_idxs(){
    return color_idxs;
}

Graph DrawingPanel::get_graph_from_map(){
    return MapData::get_graph_from_map(obstacle_map);
}

cell DrawingPanel::get_point_from_map(QPoint mouse_pos){
    QPointF scene_pos = this->mapToScene(mouse_pos);
    int scaled_x = std::round(scene_pos.x()*x_scaling);
    int scaled_y = std::round(scene_pos.y()*y_scaling);
    return cell{scaled_x, scaled_y};
}

// Map Manipulation
void DrawingPanel::update_pixmap(Map map, QImage *image){
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

void DrawingPanel::update_map(string map_type){
    Map map;
    if(map_type == OBSTACLE_MAP_ID) map = obstacle_map;
    else if(map_type == DISPLAY_MAP_ID) map = display_map;
    else if(map_type == PATH_MAP_ID) map = path_map;
    image = new QImage(map.px_width, map.px_height, QImage::Format_RGB666);
    this->update_pixmap(map, image);
    px_map = QPixmap::fromImage(*image).scaled(scene->width(),
                                               scene->height(),
                                               Qt::KeepAspectRatio);
    x_scaling = (float)obstacle_map.px_width/px_map.width();
    y_scaling = (float)obstacle_map.px_height/px_map.height();
    scene->clear();
    scene->addPixmap(px_map);
}

void DrawingPanel::update_point_on_obstacle_map(cell pt, int inflate_size, int value){
    obstacle_map.boundaries[pt.second][pt.first] = value;
    MapData::inflate_point(obstacle_map, {pt.first, pt.second}, inflate_size);
    display_map.boundaries = MapData::copy_boundaries(obstacle_map);
}

void DrawingPanel::show_path(cell start, cell goal, vector<AlgoResult> results){
    path_map = MapData::copy_map(display_map);
    if(results.empty()) return;
    else if(results.size() == 1){
        if(debug) path_map = MapData::debug_map(path_map, results[0].path, results[0].travelled, start, goal);
        else path_map = MapData::add_path_to_map(path_map, results[0].path, start, goal);
    }
    else{
        path_map = MapData::copy_map(obstacle_map);
        int path_idx = COLOR_PATH_IDX;
        for(auto r: results){
            path_map = MapData::add_path_to_map_with_value(path_map, path_idx, r.path, start, goal);
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
    this->update_map(this->PATH_MAP_ID);
    delete path_map.boundaries;
}

void DrawingPanel::remove_point_from_display(QString last_pos_str){
    auto last_pos = MapHelper::get_positon(last_pos_str.toStdString());
    if(last_pos.first >= 0 && last_pos.first < display_map.px_width && last_pos.second >= 0 && last_pos.second < display_map.px_height){
        display_map.boundaries[last_pos.second][last_pos.first] = MapData::OPEN_SPACE_INT;
        MapData::inflate_point(display_map, last_pos, MapData::POINT_SIZE);
    }
}

void DrawingPanel::add_point_to_display(QString last_pos_str, QString pos_str){
    if(map_uploaded){
        // Remove old point if applicable
        if(!last_pos_str.isEmpty() && last_pos_str != pos_str)
            this->remove_point_from_display(last_pos_str);
        // Add new point
        if(!pos_str.isEmpty()){
            auto pos = MapHelper::get_positon(pos_str.toStdString());
            if(pos.first >= 0 && pos.first < display_map.px_width && pos.second >= 0 && pos.second < display_map.px_height){
                display_map.boundaries[pos.second][pos.first] = MapData::NAV_POINT_INT;
                MapData::inflate_point(display_map, pos, MapData::POINT_SIZE);
                this->update_map(this->DISPLAY_MAP_ID);
            }
        }
    }
}

void DrawingPanel::change_map_inflation(int inflate_size){
    obstacle_map.boundaries = MapData::remove_boundary_inflation(obstacle_map);
    obstacle_map.boundaries = MapData::inflate_boundaries(obstacle_map, inflate_size);
    display_map.boundaries = MapData::copy_boundaries(obstacle_map);
    path_map.boundaries = MapData::copy_boundaries(obstacle_map);
}
