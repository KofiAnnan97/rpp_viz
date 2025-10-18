#ifndef DRAWING_PANNEL_H
#define DRAWING_PANNEL_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QObject>
#include <QColor>
#include <QPoint>

#include "app_constants.h"
#include "map_data.hpp"
#include "map_helper.hpp"

struct ColorIdx{
    int idx;
    array<int,3> rgb_vals;
    string name;
};

class DrawingPanel : public QGraphicsView{
    Q_OBJECT

    public:
        explicit DrawingPanel(QGraphicsView *parent = nullptr);

        // Setters
        void set_pen_state(bool enabled);
        void set_eraser_state(bool enabled);
        void set_map_state(bool map_state);
        void set_obstacle_map(Map map);
        void set_display_map(Map map);
        void set_debug_state(bool debug_state);

        // Getters
        bool is_map_uploaded();
        bool debug_state();
        vector<ColorIdx> get_color_idxs();
        Graph get_graph_from_map();
        cell get_point_from_map(QPoint mouse_pos);

        // Map Manipulation
        void update_pixmap(Map map, QImage *image);
        void update_map(string map_type);
        void update_point_on_obstacle_map(cell pt, int inflate_size, int value);
        void show_path(cell start, cell goal, vector<AlgoResult> results);
        void remove_point_from_display(QString last_pos_str);
        void add_point_to_display(QString last_pos_str, QString pos_str);
        void change_map_inflation(int inflate_size);

        // Map Scaling Values;
        float x_scaling = 1.0;
        float y_scaling = 1.0;

    protected:
        // void mouseMoveEvent(QDragEnterEvent* event) override;
        // void mouseReleaseEvent(QMouseEvent *event) override;
        // void mousePressEvent(QMouseEvent *event) override;

    private:
        // Graphical Elements
        QGraphicsScene *scene;
        QImage *image;
        QPixmap px_map;
        QColor empty_color = Qt::black;

        // Map Variables
        Map obstacle_map, display_map, path_map;
        bool map_uploaded = false;
        bool debug = false;
        vector<ColorIdx> color_idxs;

        // Edit Obstacles
        bool pen_enabled = false;
        bool eraser_enabled = false;
};

#endif // DRAWING_PANNEL_H
