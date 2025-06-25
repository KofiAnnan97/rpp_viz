//#include "main_window.h"
#include <gtkmm.h>
#include <iostream>

using namespace std;
using namespace Gtk;

namespace
{
Window* mw = nullptr;
Glib::RefPtr<Application> app;

void on_button_clicked(){
  if(mw) mw->set_visible(false);
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

  // Get the GtkBuilder-instantiated button, and connect a signal handler:
  /*auto pButton = builder->get_widget<Gtk::Button>("quit_button");
  if (pButton)
    pButton->signal_clicked().connect([] () { on_button_clicked(); });*/
  //pDialog->signal_hide().connect([] () { delete pDialog; });

  app->add_window(*mw);
  mw->set_visible(true);
}
} // namespace GUI

int main(int argc, char** argv){
  app = Application::create("org.gtkmm.example");
  app->signal_activate().connect([] () { on_app_activate(); });
  return app->run(argc, argv);
}