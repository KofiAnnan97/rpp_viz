//#include "main_window.h"
#include <gtkmm.h>
#include <iostream>
#include <vector>
#include <string>

#include "map_data.hpp"
#include "bfs.hpp"
#include "a_star.hpp"
#include "rrt_star.hpp"

using namespace std;
using namespace Gtk;
using namespace std::chrono;

typedef std::chrono::_V2::system_clock::time_point c_time_point;

struct AlgoResult{
  string type;
  int duration;
  vector<cell> path;
  vector<cell> travelled;
  float dist;
};

namespace
{
Glib::RefPtr<Application> app;
Window* mw = nullptr;

// Graphical Elements
Button* upload_btn = nullptr;
Button* obstacles_btn = nullptr;
Button* start_pos_btn = nullptr;
Button* goal_pos_btn = nullptr;
Button* run_algo_btn = nullptr;
Entry* start_pos_txt = nullptr;
Entry* goal_pos_txt = nullptr;
ComboBoxText* algo_type_cot = nullptr;
CheckButton* debug_ch_btn = nullptr;
TextView* results_txt_vw = nullptr;

// Variables
int inflate_size = 1;
int max_iterations = 10000;
cell start_pos = {-1,-1};
cell goal_pos = {-1,-1};
Map map;
Graph graph;
vector<AlgoResult> results;
bool debug = false;

void upload_map(){
  cout << "Upload Map" << endl;
  string test_map_yaml = "/home/eglinux/Github/rpp_viz/lib/maps/example1.yaml";
  map = MapData::get_map(test_map_yaml);

}

void update_obstacles_on_map(){
  cout << "Add/Remove Obstacles" << endl;
}

void on_start_btn_clicked(){
  cout << "Set Start Position" << endl;
  //start_pos_txt->get_buffer()->set_text("0,0");
}

void on_goal_btn_clicked(){
  cout << "Set Goal Position" << endl;
  //goal_pos_txt->get_buffer()->set_text("0,0");
}

void clear_results(){
  results.clear();
  results_txt_vw->get_buffer()->set_text("Data cleared. Running algorithms...");
}

void update_results_view(){
  string data = "";
  for(auto r : results){
    if(results.size()>1) data += std::printf("Algorithm: %s", r.type.c_str());
    data += std::printf("Duration: %d ms\n", r.duration);
    data += std::printf("Distance: %.2f\n",r.dist);
    if(debug){
      data += "Nodes: [ ";
      for(int i = 0; i<r.path.size(); i++){
        if(i == r.path.size()-1) data += std::printf("%d,%d]\n", r.path[i].first, r.path[i].second);
        else data += std::printf("%d,%d ", r.path[i].first, r.path[i].second);
      }
    }
    data += "\n";
  }
  results_txt_vw->get_buffer()->set_text(data);
}

void run_bfs(Graph g){
  auto bfs = BFS(g);
  auto start_time = high_resolution_clock::now();
  bfs.solve(graph.root, graph.end);
  auto end_time = high_resolution_clock::now();
  auto data = bfs.reconstruct_path(graph.root, graph.end);
  AlgoResult ar;
  ar.type = "BFS";
  ar.duration = duration_cast<milliseconds>(end_time- start_time).count();
  ar.path = data.first;
  ar.dist = data.second;
  if(debug) ar.travelled = bfs.get_travelled_nodes(); 
  results.push_back(ar);
}

void run_a_star(Graph g){
  auto as = AStar(g);
  auto start_time = high_resolution_clock::now();
  as.solve(graph.root, graph.end);
  auto end_time = high_resolution_clock::now();
  auto data = as.reconstruct_path(graph.root, graph.end);
  AlgoResult ar;
  ar.type = "A*";
  ar.duration = duration_cast<milliseconds>(end_time- start_time).count();
  ar.path = data.first;
  ar.dist = data.second;
  if(debug) ar.travelled = as.get_travelled_nodes(); 
  results.push_back(ar);
}

void run_rrt_star(Graph g){
  auto rrt = RRTStar(g, map.px_width, map.px_height, max_iterations);
  auto start_time = high_resolution_clock::now();
  rrt.solve(graph.root, graph.end);
  auto end_time = high_resolution_clock::now();
  auto data = rrt.reconstruct_path(graph.root, graph.end);
  AlgoResult ar;
  ar.type = "RRT*";
  ar.duration = duration_cast<milliseconds>(end_time- start_time).count();
  ar.path = data.first;
  ar.dist = data.second;
  if(debug) ar.travelled = rrt.get_travelled_nodes(); 
  results.push_back(ar);
}

void on_run_btn_clicked(){
  cout << "Run algorithm" << endl;
  
  graph = MapData::get_graph_from_map(map);
  if(!graph.is_node_valid(start_pos) || !graph.is_node_valid(goal_pos)){
    if(!graph.is_node_valid(start_pos)) cout << "Start position is not valid\n";
    if(!graph.is_node_valid(goal_pos)) cout << "Goal position is not valid\n";
    return;
  }
  else{
    graph.root = start_pos;
    graph.end = goal_pos;
    
    clear_results(); 
    string algo_type = algo_type_cot->get_active_text();
    if(strcmp(algo_type.c_str(), "BFS") == 0) run_bfs(graph);
    else if(strcmp(algo_type.c_str(), "A*") == 0) run_a_star(graph);
    else if(strcmp(algo_type.c_str(), "RRT*") == 0) run_rrt_star(graph);
    else if(strcmp(algo_type.c_str(), "All") == 0){
      run_bfs(graph);
      run_a_star(graph);
      run_rrt_star(graph);
    }
    update_results_view();
  }
}

void on_start_pos_changed(){
  cout << start_pos_txt->get_buffer()->get_text() << endl;
}

void on_goal_pos_changed(){
  cout << goal_pos_txt->get_buffer()->get_text() << endl;
}

void on_debug_toggled(){
  if(debug_ch_btn->get_active()) cout << "Debug Enabled" << endl;
  else cout << "Debug Disabled" << endl;
}

void on_algo_changed(){
  string algo_type = algo_type_cot->get_active_text();
  //cout << algo_type << endl;
  if(strcmp(algo_type.c_str(), "All") == 0) debug_ch_btn->set_visible(false);
  else debug_ch_btn->set_visible(true);
}

void on_app_activate(){
  // Create builder to extract ui from file
  auto builder = Builder::create();
  try{
    builder->add_from_file("app/main_window.ui");
  }
  catch(const Glib::FileError& ex){
    cerr << "FileError: " << ex.what() << endl;
    return;
  }
  catch(const Glib::MarkupError& ex){
    cerr << "MarkupError: " << ex.what() << endl;
    return;
  }
  catch(const Gtk::BuilderError& ex){
    cerr << "BuilderError: " << ex.what() << endl;
    return;
  }

  // Main window
  builder->get_widget("wndw_main", mw);
  mw->set_title("Path Planning Visualization");
  if(!mw){
    cerr << "Could not get the dialog" << endl;
    return;
  }

  // Button(s)
  builder->get_widget("btn_upload", upload_btn);
  if(upload_btn) upload_btn->signal_clicked().connect([] () { upload_map(); });
  //pDialog->signal_hide().connect([] () { delete pDialog; });

  builder->get_widget("btn_obstacles", obstacles_btn);
  if(obstacles_btn) obstacles_btn->signal_clicked().connect([] () { update_obstacles_on_map(); });

  builder->get_widget("btn_start_pos", start_pos_btn);
  if(start_pos_btn) start_pos_btn->signal_clicked().connect([] () { on_start_btn_clicked(); });

  builder->get_widget("btn_goal_pos", goal_pos_btn);
  if(goal_pos_btn) goal_pos_btn->signal_clicked().connect([] () { on_goal_btn_clicked(); });

  builder->get_widget("btn_run", run_algo_btn);
  if(run_algo_btn) run_algo_btn->signal_clicked().connect([] () { on_run_btn_clicked(); });

  // Textfield(s)
  builder->get_widget("txt_start_pos", start_pos_txt);
  if(start_pos_txt) start_pos_txt->signal_changed().connect([] () {on_start_pos_changed(); });

  builder->get_widget("txt_goal_pos", goal_pos_txt);
  if(goal_pos_txt) goal_pos_txt->signal_changed().connect([] () {on_goal_pos_changed(); });

  // ComboBoxText(s)
  builder->get_widget("cot_algo_type", algo_type_cot);
  if(algo_type_cot) algo_type_cot->signal_changed().connect([] () {on_algo_changed(); });

  // CheckButton(s)
  builder->get_widget("ch_btn_debug", debug_ch_btn);
  if(debug_ch_btn) debug_ch_btn->signal_toggled().connect([] () {on_debug_toggled(); });

  // TextView(s)
  builder->get_widget("txt_view_results", results_txt_vw);

  app->add_window(*mw);
  mw->set_visible(true);
}
} // namespace GUI

int main(int argc, char** argv){
  app = Application::create("org.gtkmm.example");
  app->signal_activate().connect([] () { on_app_activate(); });
  return app->run(argc, argv);
}