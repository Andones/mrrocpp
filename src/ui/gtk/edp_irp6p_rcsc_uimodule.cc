
#include <iostream>
#include <gtk/gtk.h>
#include <glib.h>
#include "ui_model.h"
#include "edp_irp6p_rcsc_uimodule.h"


edp_irp6p_rcsc::edp_irp6p_rcsc(ui_config_entry &entry)
{	
}

edp_irp6p_rcsc::~edp_irp6p_rcsc()
{
}

static edp_irp6p_rcsc *edp_postument;


extern "C" 
{ 
	void  on_combobox1_changed_postument(GtkComboBox *comboBox, gpointer userdata)  
	{
		ui_config_entry & comboEntry = *(ui_config_entry *) userdata;
		GtkBuilder & builder = (comboEntry.getBuilder());
		
		GtkScrolledWindow * scrolled = GTK_SCROLLED_WINDOW (gtk_builder_get_object(&builder, "scrolledwindow1"));

		//if the child exists, destroy it
		if (gtk_bin_get_child(GTK_BIN(scrolled))!=NULL)
		{
			GtkWidget* child = gtk_bin_get_child(GTK_BIN(scrolled));
			gtk_widget_destroy(child);
		}
		
		const gchar * ChosenFile;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox);

		switch (choice)
		{
		case 0: std::cout << "Servo algorithm window chosen" << std::endl; ChosenFile = "irp6p_servo_algorithm.xml"; break;
		case 1: std::cout << "Internal window chosen" << std::endl; ChosenFile = "irp6p_int.xml"; break;
		case 2: std::cout << "Increment window chosen" << std::endl; ChosenFile = "irp6p_inc.xml"; break;
		case 3: std::cout << "XYZ Euler ZYZ window chosen" << std::endl; ChosenFile = "irp6p_euler_xyz.xml"; break;
		case 4: std::cout << "XYZ Angle Axis window chosen" << std::endl; ChosenFile = "irp6p_axis_xyz.xml"; break;
		case 5: std::cout << "TS Euler ZYZ window chosen" << std::endl; ChosenFile = "irp6p_euler_ts.xml"; break;
		default: std::cout << "Something is not working properly!" << std::endl;
		}
		
		GtkBuilder* chosenFileBuilder = gtk_builder_new();
		GError *err = NULL;
		if (gtk_builder_add_from_file(chosenFileBuilder, ChosenFile, &err) == 0) 
		{
			fprintf (stderr, "Unable to read file %s: %s\n", ChosenFile, err->message);
			g_error_free (err);

			// TODO: throw(...)
		}
		g_assert(chosenFileBuilder);
		
		gpointer symbol;
		gtk_builder_connect_signals(chosenFileBuilder, symbol);

		GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (chosenFileBuilder, "window"));
		g_assert(chosenWindow);
		
		GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
		gtk_widget_unparent(windowWithoutParent);
		
		gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		
	}	

	void ui_module_init(ui_config_entry &entry) 
	{
		edp_postument = new edp_irp6p_rcsc(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
		const char * def_servo_algorithm = "irp6p_servo_algorithm.xml";
		const char * def_euler_xyz = "irp6p_euler_xyz.xml";
		const char * def_axis_xyz = "irp6p_axis_xyz.xml";
		const char * def_inc = "irp6p_inc.xml";
		const char * def_int = "irp6p_int.xml";
		const char * def_ts = "irp6p_euler_ts.xml";
		new ui_config_entry(ui_config_entry::EDP, "edp_irp6p_servo_algorithm", NULL, def_servo_algorithm);
		new ui_config_entry(ui_config_entry::EDP, "edp_irp6p_int", NULL, def_int);
		new ui_config_entry(ui_config_entry::EDP, "edp_irp6p_inc", NULL, def_inc);
		new ui_config_entry(ui_config_entry::EDP, "edp_irp6p_axis_xyz", NULL, def_axis_xyz);
		new ui_config_entry(ui_config_entry::EDP, "edp_irp6p_euler_zyz", NULL, def_euler_xyz);
		new ui_config_entry(ui_config_entry::EDP, "edp_irp6p_euler_ts", NULL, def_ts);
	}

	void ui_module_unload(void) 
	{
		if (edp_postument) 
		{
			delete edp_postument;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}
