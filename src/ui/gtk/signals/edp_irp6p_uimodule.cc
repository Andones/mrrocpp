
#include <iostream>

#include <gtk/gtk.h>
#include <glib.h>

#include "ui_model.h"
#include "edp_irp6p_uimodule.h"


mrrocpp::lib::BYTE servo_alg_no[7];
mrrocpp::lib::BYTE servo_par_no[7];
	
gint servo_alg_no_tmp [7];
mrrocpp::lib::BYTE servo_alg_no_output[7];
gint servo_par_no_tmp [7];
mrrocpp::lib::BYTE servo_par_no_output[7];

char buf[32];
gchar buffer[500];
double tool_vector_a[7];
double tool_vector_e[6];
double alfa, kx, ky, kz;
double wl; 
double l_eps = 0;
double irp6p_current_pos_a[8]; // pozycja biezaca
double irp6p_desired_pos_a[8]; // pozycja zadana
double irp6p_current_pos_e[7]; // pozycja biezaca
double irp6p_desired_pos_e[7]; // pozycja zadana
double irp6p_current_pos[7]; // pozycja biezaca
double irp6p_desired_pos[7]; // pozycja zadana



#include "ui/ui_ecp_r_irp6_common.h"

edp_irp6p::edp_irp6p(ui_config_entry &entry)
{
				robot_postument = new ui_common_robot(
				ui_model::instance().getConfigurator(),
				&ui_model::instance().getEcpSr()
				,mrrocpp::lib::ROBOT_IRP6_POSTUMENT
				);

}

edp_irp6p::~edp_irp6p()
{
	if (robot_postument) {
		delete robot_postument;
	}		
}

static edp_irp6p *edp_postument;


extern "C" 
{ 
	void on_read_button_clicked_postument_servo (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_postument_int (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_postument_inc (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_postument_axis_xyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_postument_euler_xyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_postument_axis_ts (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_postument_euler_ts (GtkButton * button, gpointer userdata);


	void  on_combobox1_changed_postument(GtkComboBox *comboBox, gpointer userdata)  
	{
		ui_config_entry & ChoseEntry = *(ui_config_entry *) userdata;
		GtkBuilder & builder = (ChoseEntry.getBuilder());
		
		GtkScrolledWindow * scrolled = GTK_SCROLLED_WINDOW (gtk_builder_get_object(&builder, "scrolledwindow_edp"));

		//if the child exists, destroy it
		if (gtk_bin_get_child(GTK_BIN(scrolled))!=NULL)
		{
			GtkWidget* child = gtk_bin_get_child(GTK_BIN(scrolled));
			gtk_widget_destroy(child);
		}

		gboolean isFile = 0;
		const gchar * windowName;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox); 

		if (state_postument.is_synchronised)
		{
			switch (choice)
			{
			case 0: std::cout << "Internal window chosen" << std::endl; isFile = 1; windowName = "window_int"; on_read_button_clicked_postument_int (button, userdata); break;
			case 1: std::cout << "Increment window chosen" << std::endl; isFile = 1; windowName = "window_inc"; on_read_button_clicked_postument_inc (button, userdata); break;
			case 2: std::cout << "Servo algorithm window chosen" << std::endl; isFile = 1; windowName = "window_servo"; on_read_button_clicked_postument_servo (button, userdata); break;
			case 3: std::cout << "XYZ Angle Axis window chosen" << std::endl; isFile = 1; windowName = "window_axis_xyz"; on_read_button_clicked_postument_axis_xyz (button, userdata); break;
			case 4: std::cout << "XYZ Euler ZYZ window chosen" << std::endl; isFile = 1; windowName = "window_euler_xyz"; on_read_button_clicked_postument_euler_xyz (button, userdata); break;
			case 5: std::cout << "TS Angle Axis window chosen" << std::endl; isFile = 1; windowName = "window_axis_ts"; on_read_button_clicked_postument_axis_ts (button, userdata); break;
			case 6: std::cout << "TS Euler ZYZ window chosen" << std::endl; isFile = 1; windowName = "window_euler_ts"; on_read_button_clicked_postument_euler_ts (button, userdata); break;
			default: std::cout << "Synchronizing..." << std::endl;
			}
		}
		else
		{
			switch (choice)
			{
			case 0: std::cout << "Internal window chosen" << std::endl; isFile = 1; windowName = "window_int"; on_read_button_clicked_postument_int (button, userdata); break;
			case 1: std::cout << "Increment window chosen" << std::endl; isFile = 1; windowName = "window_inc"; on_read_button_clicked_postument_inc (button, userdata); break;
			case 2: break;
			case 3: break;
			case 4: break;
			case 5: break;
			case 6: break;
			default: ;
			}
		}
		
		if (isFile)
		{	
			GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (&builder, windowName));
			g_assert(chosenWindow);
			
			GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
			gtk_widget_unparent(windowWithoutParent);
			
			gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		}
		
	}	
	
	void  on_clicked_synchronize_postument(GtkButton * button, gpointer userdata)  
	{
		ui_config_entry & comboEntry = *(ui_config_entry *) userdata;
		GtkBuilder & builder = (comboEntry.getBuilder());
		gint counter_synch;
		
		robot_postument->get_controller_state (&state_postument);
	        if(!state_postument.is_synchronised) {
	   	        GThread * synchronization_thread_postument = g_thread_create(ui_synchronize_postument, userdata, false, &error);
	        	if (synchronization_thread_postument == NULL) 
	     		{
	        		fprintf(stderr, "g_thread_create(): %s\n", error->message);
	        	}
	      }
	        robot_postument->get_controller_state (&state_postument);
	        if (state_postument.is_synchronised) {
	            gtk_widget_set_sensitive( GTK_WIDGET(button), FALSE);
	            
	            	GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&builder, "combobox1"));

					counter_synch = 2;
					gtk_combo_box_insert_text(combo, counter_synch, "Servo algorithm"); counter_synch++;
					gtk_combo_box_insert_text(combo, counter_synch, "XYZ Angle Axis"); counter_synch++;
					gtk_combo_box_insert_text(combo, counter_synch, "XYZ Euler ZYZ"); counter_synch++;
					gtk_combo_box_insert_text(combo, counter_synch, "TS Angle Axis"); counter_synch++;
					gtk_combo_box_insert_text(combo, counter_synch, "TS Euler ZYZ"); counter_synch++;

	        }
	}	

	void ui_module_init(ui_config_entry &entry) 
	{
		edp_postument = new edp_irp6p(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);

		gint counter = 0;
		GtkBuilder & builder = (entry.getBuilder());
		GtkButton * button = GTK_BUTTON (gtk_builder_get_object(&builder, "button_synchronize"));
		if (state_postument.is_synchronised) gtk_widget_set_sensitive( GTK_WIDGET(button), FALSE);
		else
		{
			GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&builder, "combobox1"));
			
			gtk_combo_box_remove_text(combo, counter); gtk_combo_box_insert_text(combo, counter, "Internal"); counter++; gtk_combo_box_insert_text(combo, counter, "Increment"); counter++;
		}
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

void *ui_synchronize_postument (gpointer userdata)
{
	ui_config_entry & comboEntry = *(ui_config_entry *) userdata;
	GtkBuilder & builder = (comboEntry.getBuilder());
	gint counter = 0;

	robot_postument->ecp->synchronise();
    
	GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&builder, "combobox1"));

	counter = 2;
	gtk_combo_box_insert_text(combo, counter, "Servo algorithm"); counter++;
	gtk_combo_box_insert_text(combo, counter, "XYZ Angle Axis"); counter++;
	gtk_combo_box_insert_text(combo, counter, "XYZ Euler ZYZ"); counter++;
	gtk_combo_box_insert_text(combo, counter, "TS Angle Axis"); counter++;
	gtk_combo_box_insert_text(combo, counter, "TS Euler ZYZ"); counter++;
	return NULL;
}



extern "C"
{
	void on_arrow_button_clicked_postument_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_servo_irp6p"));
        GtkSpinButton * spin1_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_servo_irp6p"));
        gtk_spin_button_set_value(spin1_servo_irp6p, atof(gtk_entry_get_text(entry1_servo_irp6p)));
	
        GtkEntry * entry2_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_servo_irp6p"));
        GtkSpinButton * spin2_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_servo_irp6p"));
        gtk_spin_button_set_value(spin2_servo_irp6p, atof(gtk_entry_get_text(entry2_servo_irp6p)));
	
        GtkEntry * entry3_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_servo_irp6p"));
        GtkSpinButton * spin3_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_servo_irp6p"));
        gtk_spin_button_set_value(spin3_servo_irp6p, atof(gtk_entry_get_text(entry3_servo_irp6p)));
	
        GtkEntry * entry4_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_servo_irp6p"));
        GtkSpinButton * spin4_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_servo_irp6p"));
        gtk_spin_button_set_value(spin4_servo_irp6p, atof(gtk_entry_get_text(entry4_servo_irp6p)));
	
        GtkEntry * entry5_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_servo_irp6p"));
        GtkSpinButton * spin5_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_servo_irp6p"));
        gtk_spin_button_set_value(spin5_servo_irp6p, atof(gtk_entry_get_text(entry5_servo_irp6p)));
	
        GtkEntry * entry6_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_servo_irp6p"));
        GtkSpinButton * spin6_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_servo_irp6p"));
        gtk_spin_button_set_value(spin6_servo_irp6p, atof(gtk_entry_get_text(entry6_servo_irp6p)));
	
        GtkEntry * entry7_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_servo_irp6p"));
        GtkSpinButton * spin7_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_servo_irp6p"));
        gtk_spin_button_set_value(spin7_servo_irp6p, atof(gtk_entry_get_text(entry7_servo_irp6p)));
	
        GtkEntry * entry8_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry8_servo_irp6p"));
        GtkSpinButton * spin8_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8_servo_irp6p"));
        gtk_spin_button_set_value(spin8_servo_irp6p, atof(gtk_entry_get_text(entry8_servo_irp6p)));
	
        GtkEntry * entry9_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry9_servo_irp6p"));
        GtkSpinButton * spin9_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton9_servo_irp6p"));
        gtk_spin_button_set_value(spin9_servo_irp6p, atof(gtk_entry_get_text(entry9_servo_irp6p)));
	
        GtkEntry * entry10_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry10_servo_irp6p"));
        GtkSpinButton * spin10_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton10_servo_irp6p"));
        gtk_spin_button_set_value(spin10_servo_irp6p, atof(gtk_entry_get_text(entry10_servo_irp6p)));
	
        GtkEntry * entry11_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry11_servo_irp6p"));
        GtkSpinButton * spin11_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton11_servo_irp6p"));
        gtk_spin_button_set_value(spin11_servo_irp6p, atof(gtk_entry_get_text(entry11_servo_irp6p)));
	
        GtkEntry * entry12_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry12_servo_irp6p"));
        GtkSpinButton * spin12_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton12_servo_irp6p"));
        gtk_spin_button_set_value(spin12_servo_irp6p, atof(gtk_entry_get_text(entry12_servo_irp6p)));
	
        GtkEntry * entry13_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry13_servo_irp6p"));
        GtkSpinButton * spin13_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton13_servo_irp6p"));
        gtk_spin_button_set_value(spin13_servo_irp6p, atof(gtk_entry_get_text(entry13_servo_irp6p)));
	
        GtkEntry * entry14_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry14_servo_irp6p"));
        GtkSpinButton * spin14_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton14_servo_irp6p"));
        gtk_spin_button_set_value(spin14_servo_irp6p, atof(gtk_entry_get_text(entry14_servo_irp6p)));
	
	}
	
	void on_read_button_clicked_postument_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_servo_irp6p"));
		GtkEntry * entry2_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_servo_irp6p"));
		GtkEntry * entry3_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_servo_irp6p"));
		GtkEntry * entry4_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_servo_irp6p"));
		GtkEntry * entry5_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_servo_irp6p"));
		GtkEntry * entry6_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_servo_irp6p"));
		GtkEntry * entry7_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_servo_irp6p"));
		GtkEntry * entry8_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry8_servo_irp6p"));
		GtkEntry * entry9_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry9_servo_irp6p"));
		GtkEntry * entry10_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry10_servo_irp6p"));
		GtkEntry * entry11_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry11_servo_irp6p"));
		GtkEntry * entry12_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry12_servo_irp6p"));
		GtkEntry * entry13_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry13_servo_irp6p"));
		GtkEntry * entry14_servo_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry14_servo_irp6p"));


		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
				if (state_postument.is_synchronised)  // Czy robot jest zsynchronizowany?
				{
					if (!(robot_postument->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
						printf("Blad w mechatronika get_servo_algorithm\n");
					
					gtk_entry_set_text(entry1_servo_irp6p, (const gchar*)servo_alg_no[0]);
					gtk_entry_set_text(entry2_servo_irp6p, (const gchar*)servo_par_no[0]);	
					gtk_entry_set_text(entry3_servo_irp6p, (const gchar*)servo_alg_no[1]);
					gtk_entry_set_text(entry4_servo_irp6p, (const gchar*)servo_par_no[1]);	
					gtk_entry_set_text(entry5_servo_irp6p, (const gchar*)servo_alg_no[2]);
					gtk_entry_set_text(entry6_servo_irp6p, (const gchar*)servo_par_no[2]);	
					gtk_entry_set_text(entry7_servo_irp6p, (const gchar*)servo_alg_no[3]);
					gtk_entry_set_text(entry8_servo_irp6p, (const gchar*)servo_par_no[3]);	
					gtk_entry_set_text(entry9_servo_irp6p, (const gchar*)servo_alg_no[4]);
					gtk_entry_set_text(entry10_servo_irp6p, (const gchar*)servo_par_no[4]);	
					gtk_entry_set_text(entry11_servo_irp6p, (const gchar*)servo_alg_no[5]);
					gtk_entry_set_text(entry12_servo_irp6p, (const gchar*)servo_par_no[5]);	
					gtk_entry_set_text(entry13_servo_irp6p, (const gchar*)servo_alg_no[6]);
					gtk_entry_set_text(entry14_servo_irp6p, (const gchar*)servo_par_no[6]);	
					
				} else
				{
					std::cout << "Robot is not synchronized" << std::endl;
				}
			}
	}
	
	void on_set_button_clicked_postument_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());

		GtkSpinButton * spin1_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_servo_irp6p"));
		GtkSpinButton * spin2_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_servo_irp6p"));
		GtkSpinButton * spin3_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_servo_irp6p"));
		GtkSpinButton * spin4_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_servo_irp6p"));
		GtkSpinButton * spin5_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_servo_irp6p"));
		GtkSpinButton * spin6_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_servo_irp6p"));
		GtkSpinButton * spin7_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_servo_irp6p"));
		GtkSpinButton * spin8_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8_servo_irp6p"));
		GtkSpinButton * spin9_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton9_servo_irp6p"));
		GtkSpinButton * spin10_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton10_servo_irp6p"));
		GtkSpinButton * spin11_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton11_servo_irp6p"));
		GtkSpinButton * spin12_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton12_servo_irp6p"));
		GtkSpinButton * spin13_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton13_servo_irp6p"));
		GtkSpinButton * spin14_servo_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton14_servo_irp6p"));

 		
 		if (state_postument.is_synchronised)
		{
			servo_alg_no_tmp[0] = gtk_spin_button_get_value_as_int(spin1_servo_irp6p);
			servo_par_no_tmp[0] = gtk_spin_button_get_value_as_int(spin2_servo_irp6p);
			servo_alg_no_tmp[1] = gtk_spin_button_get_value_as_int(spin3_servo_irp6p);
			servo_par_no_tmp[1] = gtk_spin_button_get_value_as_int(spin4_servo_irp6p);
			servo_alg_no_tmp[2] = gtk_spin_button_get_value_as_int(spin5_servo_irp6p);
			servo_par_no_tmp[2] = gtk_spin_button_get_value_as_int(spin6_servo_irp6p);
			servo_alg_no_tmp[3] = gtk_spin_button_get_value_as_int(spin7_servo_irp6p);
			servo_par_no_tmp[3] = gtk_spin_button_get_value_as_int(spin8_servo_irp6p);
			servo_alg_no_tmp[4] = gtk_spin_button_get_value_as_int(spin9_servo_irp6p);
			servo_par_no_tmp[4] = gtk_spin_button_get_value_as_int(spin10_servo_irp6p);
			servo_alg_no_tmp[5] = gtk_spin_button_get_value_as_int(spin11_servo_irp6p);
			servo_par_no_tmp[5] = gtk_spin_button_get_value_as_int(spin12_servo_irp6p);
			servo_alg_no_tmp[6] = gtk_spin_button_get_value_as_int(spin13_servo_irp6p);
			servo_par_no_tmp[6] = gtk_spin_button_get_value_as_int(spin14_servo_irp6p);


		for(int i=0; i<7; i++)
		{
			servo_alg_no_output[i] = mrrocpp::lib::BYTE(servo_alg_no_tmp[i]);
			servo_par_no_output[i] = mrrocpp::lib::BYTE(servo_par_no_tmp[i]);
		}

		// zlecenie wykonania ruchu
		robot_postument->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

	}
	else
	{
		std::cout << "Robot is not synchronized" << std::endl;
	}
 		
	}
}


extern "C"
{
	void on_arrow_button_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_int_irp6p"));
        GtkSpinButton * spin1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_irp6p"));
        gtk_spin_button_set_value(spin1_int_irp6p, atof(gtk_entry_get_text(entry1_int_irp6p)));
	
        GtkEntry * entry2_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_int_irp6p"));
        GtkSpinButton * spin2_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_int_irp6p"));
        gtk_spin_button_set_value(spin2_int_irp6p, atof(gtk_entry_get_text(entry2_int_irp6p)));
	
        GtkEntry * entry3_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_int_irp6p"));
        GtkSpinButton * spin3_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_int_irp6p"));
        gtk_spin_button_set_value(spin3_int_irp6p, atof(gtk_entry_get_text(entry3_int_irp6p)));
	
        GtkEntry * entry4_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_int_irp6p"));
        GtkSpinButton * spin4_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_int_irp6p"));
        gtk_spin_button_set_value(spin4_int_irp6p, atof(gtk_entry_get_text(entry4_int_irp6p)));
	
        GtkEntry * entry5_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_int_irp6p"));
        GtkSpinButton * spin5_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_int_irp6p"));
        gtk_spin_button_set_value(spin5_int_irp6p, atof(gtk_entry_get_text(entry5_int_irp6p)));
	
        GtkEntry * entry6_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_int_irp6p"));
        GtkSpinButton * spin6_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_int_irp6p"));
        gtk_spin_button_set_value(spin6_int_irp6p, atof(gtk_entry_get_text(entry6_int_irp6p)));
	
        GtkEntry * entry7_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_int_irp6p"));
        GtkSpinButton * spin7_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_int_irp6p"));
        gtk_spin_button_set_value(spin7_int_irp6p, atof(gtk_entry_get_text(entry7_int_irp6p)));
	
	}
	
	void on_read_button_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_int_irp6p"));
		GtkEntry * entry2_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_int_irp6p"));
		GtkEntry * entry3_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_int_irp6p"));
		GtkEntry * entry4_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_int_irp6p"));
		GtkEntry * entry5_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_int_irp6p"));
		GtkEntry * entry6_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_int_irp6p"));
		GtkEntry * entry7_int_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_int_irp6p"));
	
 		
		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_postument->read_joints(irp6p_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[0]);
					gtk_entry_set_text(entry1_int_irp6p, buf);
					irp6p_desired_pos[0] = irp6p_current_pos[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[1]);
					gtk_entry_set_text(entry2_int_irp6p, buf);
					irp6p_desired_pos[1] = irp6p_current_pos[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[2]);
					gtk_entry_set_text(entry3_int_irp6p, buf);
					irp6p_desired_pos[2] = irp6p_current_pos[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[3]);
					gtk_entry_set_text(entry4_int_irp6p, buf);
					irp6p_desired_pos[3] = irp6p_current_pos[3];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[4]);
					gtk_entry_set_text(entry5_int_irp6p, buf);
					irp6p_desired_pos[4] = irp6p_current_pos[4];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[5]);
					gtk_entry_set_text(entry6_int_irp6p, buf);
					irp6p_desired_pos[5] = irp6p_current_pos[5];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[6]);
					gtk_entry_set_text(entry7_int_irp6p, buf);
					irp6p_desired_pos[6] = irp6p_current_pos[6];				
		
 				
 				for (int i = 0; i < 7; i++)
				irp6p_desired_pos[i] = irp6p_current_pos[i];		
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_irp6p"));
 		GtkSpinButton * spin2_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_int_irp6p"));
 		GtkSpinButton * spin3_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_int_irp6p"));
 		GtkSpinButton * spin4_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_int_irp6p"));
 		GtkSpinButton * spin5_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_int_irp6p"));
 		GtkSpinButton * spin6_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_int_irp6p"));
 		GtkSpinButton * spin7_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_int_irp6p"));
 	    

		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
				irp6p_desired_pos[0] = gtk_spin_button_get_value(spin1_int_irp6p);
				irp6p_desired_pos[1] = gtk_spin_button_get_value(spin2_int_irp6p);
				irp6p_desired_pos[2] = gtk_spin_button_get_value(spin3_int_irp6p);
				irp6p_desired_pos[3] = gtk_spin_button_get_value(spin4_int_irp6p);
				irp6p_desired_pos[4] = gtk_spin_button_get_value(spin5_int_irp6p);
				irp6p_desired_pos[5] = gtk_spin_button_get_value(spin6_int_irp6p);
				irp6p_desired_pos[6] = gtk_spin_button_get_value(spin7_int_irp6p);
	    
			
			robot_postument->move_joints(irp6p_desired_pos);
			
			 if (state_postument.is_synchronised) {
				gtk_spin_button_set_value(spin1_int_irp6p, irp6p_desired_pos[0]);
				gtk_spin_button_set_value(spin2_int_irp6p, irp6p_desired_pos[1]);
				gtk_spin_button_set_value(spin3_int_irp6p, irp6p_desired_pos[2]);
				gtk_spin_button_set_value(spin4_int_irp6p, irp6p_desired_pos[3]);
				gtk_spin_button_set_value(spin5_int_irp6p, irp6p_desired_pos[4]);
				gtk_spin_button_set_value(spin6_int_irp6p, irp6p_desired_pos[5]);
				gtk_spin_button_set_value(spin7_int_irp6p, irp6p_desired_pos[6]);
	  
			 }
		}
		on_read_button_clicked_postument_int (button, userdata);

	}
	
	void on_export_button_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_irp6p"));
 		GtkSpinButton * spin2_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_int_irp6p"));
 		GtkSpinButton * spin3_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_int_irp6p"));
 		GtkSpinButton * spin4_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_int_irp6p"));
 		GtkSpinButton * spin5_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_int_irp6p"));
 		GtkSpinButton * spin6_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_int_irp6p"));
 		GtkSpinButton * spin7_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_int_irp6p"));
 	
 		sprintf(buffer, "edp_irp6p INTERNAL position  %.3f %.3f %.3f %.3f %.3f %.3f %.3f" 
 		, gtk_spin_button_get_value(spin1_int_irp6p), gtk_spin_button_get_value(spin2_int_irp6p), gtk_spin_button_get_value(spin3_int_irp6p), gtk_spin_button_get_value(spin4_int_irp6p), gtk_spin_button_get_value(spin5_int_irp6p), gtk_spin_button_get_value(spin6_int_irp6p), gtk_spin_button_get_value(spin7_int_irp6p));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_"));
        gtk_spin_button_set_value(spin1_int_, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin2_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_int_"));
        gtk_spin_button_set_value(spin2_int_, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin3_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_int_"));
        gtk_spin_button_set_value(spin3_int_, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin4_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_int_"));
        gtk_spin_button_set_value(spin4_int_, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin5_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_int_"));
        gtk_spin_button_set_value(spin5_int_, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin6_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_int_"));
        gtk_spin_button_set_value(spin6_int_, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin7_int_ = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_int_"));
        gtk_spin_button_set_value(spin7_int_, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	

	void on_button1_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_irp6p"));
        gtk_spin_button_set_value(spin1_int_irp6p, gtk_spin_button_get_value(spin1_int_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}
	
	void on_button2_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_int_irp6p"));
        gtk_spin_button_set_value(spin1_int_irp6p, gtk_spin_button_get_value(spin1_int_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}   

	void on_button3_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin2_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_int_irp6p"));
        gtk_spin_button_set_value(spin2_int_irp6p, gtk_spin_button_get_value(spin2_int_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}
	
	void on_button4_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin2_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_int_irp6p"));
        gtk_spin_button_set_value(spin2_int_irp6p, gtk_spin_button_get_value(spin2_int_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}   

	void on_button5_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin3_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_int_irp6p"));
        gtk_spin_button_set_value(spin3_int_irp6p, gtk_spin_button_get_value(spin3_int_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}
	
	void on_button6_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin3_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_int_irp6p"));
        gtk_spin_button_set_value(spin3_int_irp6p, gtk_spin_button_get_value(spin3_int_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}   

	void on_button7_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin4_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_int_irp6p"));
        gtk_spin_button_set_value(spin4_int_irp6p, gtk_spin_button_get_value(spin4_int_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}
	
	void on_button8_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin4_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_int_irp6p"));
        gtk_spin_button_set_value(spin4_int_irp6p, gtk_spin_button_get_value(spin4_int_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}   

	void on_button9_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin5_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_int_irp6p"));
        gtk_spin_button_set_value(spin5_int_irp6p, gtk_spin_button_get_value(spin5_int_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}
	
	void on_button10_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin5_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_int_irp6p"));
        gtk_spin_button_set_value(spin5_int_irp6p, gtk_spin_button_get_value(spin5_int_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}   

	void on_button11_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin6_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_int_irp6p"));
        gtk_spin_button_set_value(spin6_int_irp6p, gtk_spin_button_get_value(spin6_int_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}
	
	void on_button12_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin6_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_int_irp6p"));
        gtk_spin_button_set_value(spin6_int_irp6p, gtk_spin_button_get_value(spin6_int_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}   

	void on_button13_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin7_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_int_irp6p"));
        gtk_spin_button_set_value(spin7_int_irp6p, gtk_spin_button_get_value(spin7_int_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}
	
	void on_button14_clicked_postument_int (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_int_irp6p"));
        GtkSpinButton * spin7_int_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_int_irp6p"));
        gtk_spin_button_set_value(spin7_int_irp6p, gtk_spin_button_get_value(spin7_int_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_int_irp6p));
 		
 		on_execute_button_clicked_postument_int (button, userdata);
 	}   

}


extern "C"
{
	void on_arrow_button_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_inc_irp6p"));
        GtkSpinButton * spin1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_irp6p"));
        gtk_spin_button_set_value(spin1_inc_irp6p, atof(gtk_entry_get_text(entry1_inc_irp6p)));
	
        GtkEntry * entry2_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_inc_irp6p"));
        GtkSpinButton * spin2_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_inc_irp6p"));
        gtk_spin_button_set_value(spin2_inc_irp6p, atof(gtk_entry_get_text(entry2_inc_irp6p)));
	
        GtkEntry * entry3_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_inc_irp6p"));
        GtkSpinButton * spin3_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_inc_irp6p"));
        gtk_spin_button_set_value(spin3_inc_irp6p, atof(gtk_entry_get_text(entry3_inc_irp6p)));
	
        GtkEntry * entry4_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_inc_irp6p"));
        GtkSpinButton * spin4_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_inc_irp6p"));
        gtk_spin_button_set_value(spin4_inc_irp6p, atof(gtk_entry_get_text(entry4_inc_irp6p)));
	
        GtkEntry * entry5_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_inc_irp6p"));
        GtkSpinButton * spin5_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_inc_irp6p"));
        gtk_spin_button_set_value(spin5_inc_irp6p, atof(gtk_entry_get_text(entry5_inc_irp6p)));
	
        GtkEntry * entry6_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_inc_irp6p"));
        GtkSpinButton * spin6_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_inc_irp6p"));
        gtk_spin_button_set_value(spin6_inc_irp6p, atof(gtk_entry_get_text(entry6_inc_irp6p)));
	
        GtkEntry * entry7_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_inc_irp6p"));
        GtkSpinButton * spin7_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_inc_irp6p"));
        gtk_spin_button_set_value(spin7_inc_irp6p, atof(gtk_entry_get_text(entry7_inc_irp6p)));
	
	}
	
	void on_read_button_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_inc_irp6p"));
		GtkEntry * entry2_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_inc_irp6p"));
		GtkEntry * entry3_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_inc_irp6p"));
		GtkEntry * entry4_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_inc_irp6p"));
		GtkEntry * entry5_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_inc_irp6p"));
		GtkEntry * entry6_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_inc_irp6p"));
		GtkEntry * entry7_inc_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_inc_irp6p"));
	
 		
		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_postument->read_motors(irp6p_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[0]);
					gtk_entry_set_text(entry1_inc_irp6p, buf);
					irp6p_desired_pos[0] = irp6p_current_pos[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[1]);
					gtk_entry_set_text(entry2_inc_irp6p, buf);
					irp6p_desired_pos[1] = irp6p_current_pos[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[2]);
					gtk_entry_set_text(entry3_inc_irp6p, buf);
					irp6p_desired_pos[2] = irp6p_current_pos[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[3]);
					gtk_entry_set_text(entry4_inc_irp6p, buf);
					irp6p_desired_pos[3] = irp6p_current_pos[3];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[4]);
					gtk_entry_set_text(entry5_inc_irp6p, buf);
					irp6p_desired_pos[4] = irp6p_current_pos[4];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[5]);
					gtk_entry_set_text(entry6_inc_irp6p, buf);
					irp6p_desired_pos[5] = irp6p_current_pos[5];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos[6]);
					gtk_entry_set_text(entry7_inc_irp6p, buf);
					irp6p_desired_pos[6] = irp6p_current_pos[6];				
	
 				
 				for (int i = 0; i < 7; i++)
				irp6p_desired_pos[i] = irp6p_current_pos[i];			
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_irp6p"));
 		GtkSpinButton * spin2_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_inc_irp6p"));
 		GtkSpinButton * spin3_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_inc_irp6p"));
 		GtkSpinButton * spin4_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_inc_irp6p"));
 		GtkSpinButton * spin5_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_inc_irp6p"));
 		GtkSpinButton * spin6_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_inc_irp6p"));
 		GtkSpinButton * spin7_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_inc_irp6p"));
 	    

		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) {
				irp6p_desired_pos[0] = gtk_spin_button_get_value(spin1_inc_irp6p);
				irp6p_desired_pos[1] = gtk_spin_button_get_value(spin2_inc_irp6p);
				irp6p_desired_pos[2] = gtk_spin_button_get_value(spin3_inc_irp6p);
				irp6p_desired_pos[3] = gtk_spin_button_get_value(spin4_inc_irp6p);
				irp6p_desired_pos[4] = gtk_spin_button_get_value(spin5_inc_irp6p);
				irp6p_desired_pos[5] = gtk_spin_button_get_value(spin6_inc_irp6p);
				irp6p_desired_pos[6] = gtk_spin_button_get_value(spin7_inc_irp6p);
	    
			} else {
				 for (int i = 0; i < 7; i++)
				 {
		         	 irp6p_desired_pos[i] = 0.0;
	        	 }
	   		 }
			
			robot_postument->move_motors(irp6p_desired_pos);
			
			 if (state_postument.is_synchronised) {
				gtk_spin_button_set_value(spin1_inc_irp6p, irp6p_desired_pos[0]);
				gtk_spin_button_set_value(spin2_inc_irp6p, irp6p_desired_pos[1]);
				gtk_spin_button_set_value(spin3_inc_irp6p, irp6p_desired_pos[2]);
				gtk_spin_button_set_value(spin4_inc_irp6p, irp6p_desired_pos[3]);
				gtk_spin_button_set_value(spin5_inc_irp6p, irp6p_desired_pos[4]);
				gtk_spin_button_set_value(spin6_inc_irp6p, irp6p_desired_pos[5]);
				gtk_spin_button_set_value(spin7_inc_irp6p, irp6p_desired_pos[6]);
	  
			 }
		}
		on_read_button_clicked_postument_inc (button, userdata);

	}
	
	void on_export_button_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_irp6p"));
 		GtkSpinButton * spin2_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_inc_irp6p"));
 		GtkSpinButton * spin3_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_inc_irp6p"));
 		GtkSpinButton * spin4_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_inc_irp6p"));
 		GtkSpinButton * spin5_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_inc_irp6p"));
 		GtkSpinButton * spin6_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_inc_irp6p"));
 		GtkSpinButton * spin7_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_inc_irp6p"));
 	
 		sprintf(buffer, "edp_irp6p INCREMENT position  %.3f %.3f %.3f %.3f %.3f %.3f %.3f" 
 		, gtk_spin_button_get_value(spin1_inc_irp6p), gtk_spin_button_get_value(spin2_inc_irp6p), gtk_spin_button_get_value(spin3_inc_irp6p), gtk_spin_button_get_value(spin4_inc_irp6p), gtk_spin_button_get_value(spin5_inc_irp6p), gtk_spin_button_get_value(spin6_inc_irp6p), gtk_spin_button_get_value(spin7_inc_irp6p));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_irp6p"));
        gtk_spin_button_set_value(spin1_inc_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin2_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_inc_irp6p"));
        gtk_spin_button_set_value(spin2_inc_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin3_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_inc_irp6p"));
        gtk_spin_button_set_value(spin3_inc_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin4_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_inc_irp6p"));
        gtk_spin_button_set_value(spin4_inc_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin5_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_inc_irp6p"));
        gtk_spin_button_set_value(spin5_inc_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin6_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_inc_irp6p"));
        gtk_spin_button_set_value(spin6_inc_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin7_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_inc_irp6p"));
        gtk_spin_button_set_value(spin7_inc_irp6p, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	
	
	

	void on_button1_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_irp6p"));
        gtk_spin_button_set_value(spin1_inc_irp6p, gtk_spin_button_get_value(spin1_inc_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
		on_execute_button_clicked_postument_inc (button, userdata); 	
 	}
	
	void on_button2_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_inc_irp6p"));
        gtk_spin_button_set_value(spin1_inc_irp6p, gtk_spin_button_get_value(spin1_inc_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
 		on_execute_button_clicked_postument_inc (button, userdata);
 	}    

	void on_button3_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin2_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_inc_irp6p"));
        gtk_spin_button_set_value(spin2_inc_irp6p, gtk_spin_button_get_value(spin2_inc_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
		on_execute_button_clicked_postument_inc (button, userdata); 	
 	}
	
	void on_button4_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin2_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_inc_irp6p"));
        gtk_spin_button_set_value(spin2_inc_irp6p, gtk_spin_button_get_value(spin2_inc_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
 		on_execute_button_clicked_postument_inc (button, userdata);
 	}    

	void on_button5_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin3_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_inc_irp6p"));
        gtk_spin_button_set_value(spin3_inc_irp6p, gtk_spin_button_get_value(spin3_inc_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
		on_execute_button_clicked_postument_inc (button, userdata); 	
 	}
	
	void on_button6_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin3_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_inc_irp6p"));
        gtk_spin_button_set_value(spin3_inc_irp6p, gtk_spin_button_get_value(spin3_inc_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
 		on_execute_button_clicked_postument_inc (button, userdata);
 	}    

	void on_button7_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin4_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_inc_irp6p"));
        gtk_spin_button_set_value(spin4_inc_irp6p, gtk_spin_button_get_value(spin4_inc_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
		on_execute_button_clicked_postument_inc (button, userdata); 	
 	}
	
	void on_button8_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin4_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_inc_irp6p"));
        gtk_spin_button_set_value(spin4_inc_irp6p, gtk_spin_button_get_value(spin4_inc_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
 		on_execute_button_clicked_postument_inc (button, userdata);
 	}    

	void on_button9_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin5_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_inc_irp6p"));
        gtk_spin_button_set_value(spin5_inc_irp6p, gtk_spin_button_get_value(spin5_inc_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
		on_execute_button_clicked_postument_inc (button, userdata); 	
 	}
	
	void on_button10_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin5_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_inc_irp6p"));
        gtk_spin_button_set_value(spin5_inc_irp6p, gtk_spin_button_get_value(spin5_inc_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
 		on_execute_button_clicked_postument_inc (button, userdata);
 	}    

	void on_button11_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin6_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_inc_irp6p"));
        gtk_spin_button_set_value(spin6_inc_irp6p, gtk_spin_button_get_value(spin6_inc_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
		on_execute_button_clicked_postument_inc (button, userdata); 	
 	}
	
	void on_button12_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin6_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_inc_irp6p"));
        gtk_spin_button_set_value(spin6_inc_irp6p, gtk_spin_button_get_value(spin6_inc_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
 		on_execute_button_clicked_postument_inc (button, userdata);
 	}    

	void on_button13_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin7_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_inc_irp6p"));
        gtk_spin_button_set_value(spin7_inc_irp6p, gtk_spin_button_get_value(spin7_inc_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
		on_execute_button_clicked_postument_inc (button, userdata); 	
 	}
	
	void on_button14_clicked_postument_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_inc_irp6p"));
        GtkSpinButton * spin7_inc_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_inc_irp6p"));
        gtk_spin_button_set_value(spin7_inc_irp6p, gtk_spin_button_get_value(spin7_inc_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_inc_irp6p));
 	
 		on_execute_button_clicked_postument_inc (button, userdata);
 	}    

}


extern "C"
{
	void on_arrow_button_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_axis_xyz_irp6p"));
        GtkSpinButton * spin1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin1_axis_xyz_irp6p, atof(gtk_entry_get_text(entry1_axis_xyz_irp6p)));
	
        GtkEntry * entry2_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_axis_xyz_irp6p"));
        GtkSpinButton * spin2_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin2_axis_xyz_irp6p, atof(gtk_entry_get_text(entry2_axis_xyz_irp6p)));
	
        GtkEntry * entry3_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_axis_xyz_irp6p"));
        GtkSpinButton * spin3_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin3_axis_xyz_irp6p, atof(gtk_entry_get_text(entry3_axis_xyz_irp6p)));
	
        GtkEntry * entry4_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_axis_xyz_irp6p"));
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin4_axis_xyz_irp6p, atof(gtk_entry_get_text(entry4_axis_xyz_irp6p)));
	
        GtkEntry * entry5_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin5_axis_xyz_irp6p, atof(gtk_entry_get_text(entry5_axis_xyz_irp6p)));
	
        GtkEntry * entry6_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin6_axis_xyz_irp6p, atof(gtk_entry_get_text(entry6_axis_xyz_irp6p)));
	
        GtkEntry * entry7_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_axis_xyz_irp6p"));
        GtkSpinButton * spin7_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin7_axis_xyz_irp6p, atof(gtk_entry_get_text(entry7_axis_xyz_irp6p)));
	
        GtkEntry * entry8_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry8_axis_xyz_irp6p"));
        GtkSpinButton * spin8_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin8_axis_xyz_irp6p, atof(gtk_entry_get_text(entry8_axis_xyz_irp6p)));
	
	}
	
	void on_read_button_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_axis_xyz_irp6p"));
		GtkEntry * entry2_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_axis_xyz_irp6p"));
		GtkEntry * entry3_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_axis_xyz_irp6p"));
		GtkEntry * entry4_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_axis_xyz_irp6p"));
		GtkEntry * entry5_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_axis_xyz_irp6p"));
		GtkEntry * entry6_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_axis_xyz_irp6p"));
		GtkEntry * entry7_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_axis_xyz_irp6p"));
		GtkEntry * entry8_axis_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry8_axis_xyz_irp6p"));
	
 		
		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_postument->read_xyz_angle_axis(irp6p_current_pos_a))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
				alfa = sqrt(irp6p_current_pos_a[3]*irp6p_current_pos_a[3]
				+irp6p_current_pos_a[4]*irp6p_current_pos_a[4]
				+irp6p_current_pos_a[5]*irp6p_current_pos_a[5]);
					
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_a[0]);
					gtk_entry_set_text(entry1_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[0] = irp6p_current_pos_a[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_a[1]);
					gtk_entry_set_text(entry2_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[1] = irp6p_current_pos_a[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_a[2]);
					gtk_entry_set_text(entry3_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[2] = irp6p_current_pos_a[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_a[3]/alfa);
					gtk_entry_set_text(entry4_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[3] = irp6p_current_pos_a[3]/alfa;
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_a[4]/alfa);
					gtk_entry_set_text(entry5_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[4] = irp6p_current_pos_a[4]/alfa;
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_a[5]/alfa);
					gtk_entry_set_text(entry6_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[5] = irp6p_current_pos_a[5]/alfa;
					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry7_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[6] = alfa;							
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_a[7]);
					gtk_entry_set_text(entry8_axis_xyz_irp6p, buf);
					irp6p_desired_pos_a[7] = irp6p_current_pos_a[7];				
				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_axis_xyz_irp6p"));
 		GtkSpinButton * spin2_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_axis_xyz_irp6p"));
 		GtkSpinButton * spin3_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_axis_xyz_irp6p"));
 		GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
 		GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
 		GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
 		GtkSpinButton * spin7_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_axis_xyz_irp6p"));
 		GtkSpinButton * spin8_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8_axis_xyz_irp6p"));
 	    

		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
				irp6p_desired_pos_a[0] = gtk_spin_button_get_value(spin1_axis_xyz_irp6p);
				irp6p_desired_pos_a[1] = gtk_spin_button_get_value(spin2_axis_xyz_irp6p);
				irp6p_desired_pos_a[2] = gtk_spin_button_get_value(spin3_axis_xyz_irp6p);
				irp6p_desired_pos_a[3] = gtk_spin_button_get_value(spin4_axis_xyz_irp6p);
				irp6p_desired_pos_a[4] = gtk_spin_button_get_value(spin5_axis_xyz_irp6p);
				irp6p_desired_pos_a[5] = gtk_spin_button_get_value(spin6_axis_xyz_irp6p);
				irp6p_desired_pos_a[6] = gtk_spin_button_get_value(spin7_axis_xyz_irp6p);
				irp6p_desired_pos_a[7] = gtk_spin_button_get_value(spin8_axis_xyz_irp6p);
	    
 		
 			// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
			for(int i=3; i<8; i++)
			{
					irp6p_desired_pos_a[i] *= irp6p_desired_pos_a[6];
			}
			
			robot_postument->move_xyz_angle_axis(irp6p_desired_pos_a);
			
			 if (state_postument.is_synchronised) {
				gtk_spin_button_set_value(spin1_axis_xyz_irp6p, irp6p_desired_pos_a[0]);
				gtk_spin_button_set_value(spin2_axis_xyz_irp6p, irp6p_desired_pos_a[1]);
				gtk_spin_button_set_value(spin3_axis_xyz_irp6p, irp6p_desired_pos_a[2]);
				gtk_spin_button_set_value(spin4_axis_xyz_irp6p, irp6p_desired_pos_a[3]);
				gtk_spin_button_set_value(spin5_axis_xyz_irp6p, irp6p_desired_pos_a[4]);
				gtk_spin_button_set_value(spin6_axis_xyz_irp6p, irp6p_desired_pos_a[5]);
				gtk_spin_button_set_value(spin7_axis_xyz_irp6p, irp6p_desired_pos_a[6]);
				gtk_spin_button_set_value(spin8_axis_xyz_irp6p, irp6p_desired_pos_a[7]);
	  
			 }
		}
		on_read_button_clicked_postument_axis_xyz (button, userdata);

	}
	

	void on_button1_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin1_axis_xyz_irp6p, gtk_spin_button_get_value(spin1_axis_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
 	}
	
	void on_button2_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin1_axis_xyz_irp6p, gtk_spin_button_get_value(spin1_axis_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
}   

	void on_button3_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin2_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin2_axis_xyz_irp6p, gtk_spin_button_get_value(spin2_axis_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
 	}
	
	void on_button4_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin2_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin2_axis_xyz_irp6p, gtk_spin_button_get_value(spin2_axis_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
}   

	void on_button5_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin3_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin3_axis_xyz_irp6p, gtk_spin_button_get_value(spin3_axis_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
 	}
	
	void on_button6_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin3_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin3_axis_xyz_irp6p, gtk_spin_button_get_value(spin3_axis_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
}   

	void on_button7_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
               
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}
	
	void on_button8_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}  

	void on_button9_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
               
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}
	
	void on_button10_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}  

	void on_button11_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
               
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}
	
	void on_button12_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}  

	void on_button13_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin7_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin7_axis_xyz_irp6p, gtk_spin_button_get_value(spin7_axis_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}
	
	void on_button14_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin7_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin7_axis_xyz_irp6p, gtk_spin_button_get_value(spin7_axis_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}   

	void on_button15_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin8_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin8_axis_xyz_irp6p, gtk_spin_button_get_value(spin8_axis_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}
	
	void on_button16_clicked_postument_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_axis_xyz_irp6p"));
        GtkSpinButton * spin8_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton8_axis_xyz_irp6p"));
        gtk_spin_button_set_value(spin8_axis_xyz_irp6p, gtk_spin_button_get_value(spin8_axis_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_irp6p));
        
        GtkSpinButton * spin4_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_xyz_irp6p"));
        GtkSpinButton * spin5_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_xyz_irp6p"));
        GtkSpinButton * spin6_axis_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_xyz_irp6p"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_irp6p)*gtk_spin_button_get_value(spin4_axis_xyz_irp6p) + gtk_spin_button_get_value(spin5_axis_xyz_irp6p)*gtk_spin_button_get_value(spin5_axis_xyz_irp6p) + gtk_spin_button_get_value(spin6_axis_xyz_irp6p)*gtk_spin_button_get_value(spin6_axis_xyz_irp6p));
		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4_axis_xyz_irp6p, gtk_spin_button_get_value(spin4_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin5_axis_xyz_irp6p, gtk_spin_button_get_value(spin5_axis_xyz_irp6p) / wl);
			gtk_spin_button_set_value(spin6_axis_xyz_irp6p, gtk_spin_button_get_value(spin6_axis_xyz_irp6p) / wl);
		}
		
		on_execute_button_clicked_postument_axis_xyz (button, userdata);
	}   

}


extern "C"
{
	void on_arrow_button_clicked_postument_axis_ts (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_axis_ts_irp6p"));
        GtkSpinButton * spin1_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_axis_ts_irp6p"));
        gtk_spin_button_set_value(spin1_axis_ts_irp6p, atof(gtk_entry_get_text(entry1_axis_ts_irp6p)));
	
        GtkEntry * entry2_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_axis_ts_irp6p"));
        GtkSpinButton * spin2_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_axis_ts_irp6p"));
        gtk_spin_button_set_value(spin2_axis_ts_irp6p, atof(gtk_entry_get_text(entry2_axis_ts_irp6p)));
	
        GtkEntry * entry3_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_axis_ts_irp6p"));
        GtkSpinButton * spin3_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_axis_ts_irp6p"));
        gtk_spin_button_set_value(spin3_axis_ts_irp6p, atof(gtk_entry_get_text(entry3_axis_ts_irp6p)));
	
        GtkEntry * entry4_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_axis_ts_irp6p"));
        GtkSpinButton * spin4_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_ts_irp6p"));
        gtk_spin_button_set_value(spin4_axis_ts_irp6p, atof(gtk_entry_get_text(entry4_axis_ts_irp6p)));
	
        GtkEntry * entry5_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_axis_ts_irp6p"));
        GtkSpinButton * spin5_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_ts_irp6p"));
        gtk_spin_button_set_value(spin5_axis_ts_irp6p, atof(gtk_entry_get_text(entry5_axis_ts_irp6p)));
	
        GtkEntry * entry6_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_axis_ts_irp6p"));
        GtkSpinButton * spin6_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_ts_irp6p"));
        gtk_spin_button_set_value(spin6_axis_ts_irp6p, atof(gtk_entry_get_text(entry6_axis_ts_irp6p)));
	
        GtkEntry * entry7_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_axis_ts_irp6p"));
        GtkSpinButton * spin7_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_axis_ts_irp6p"));
        gtk_spin_button_set_value(spin7_axis_ts_irp6p, atof(gtk_entry_get_text(entry7_axis_ts_irp6p)));
	
 	}
	
	void on_read_button_clicked_postument_axis_ts (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_axis_ts_irp6p"));
		GtkEntry * entry2_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_axis_ts_irp6p"));
		GtkEntry * entry3_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_axis_ts_irp6p"));
		GtkEntry * entry4_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_axis_ts_irp6p"));
		GtkEntry * entry5_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_axis_ts_irp6p"));
		GtkEntry * entry6_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_axis_ts_irp6p"));
		GtkEntry * entry7_axis_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_axis_ts_irp6p"));
	
 		
		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_postument->read_tool_xyz_angle_axis(tool_vector_a))) // Odczyt polozenia walow silnikow
					printf("Blad w read external\n");
					
				alfa = sqrt(tool_vector_a[3]*tool_vector_a[3]
				+tool_vector_a[4]*tool_vector_a[4]
				+tool_vector_a[5]*tool_vector_a[5]);
				
				if (alfa==0){
					tool_vector_a[3] = -1;
					tool_vector_a[4] = 0;
					tool_vector_a[5] = 0;
				}
				else{
					tool_vector_a[3] = tool_vector_a[3]/alfa;
					tool_vector_a[4] = tool_vector_a[4]/alfa;
					tool_vector_a[5] = tool_vector_a[5]/alfa;
				}
					
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[0]);
					gtk_entry_set_text(entry1_axis_ts_irp6p, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[1]);
					gtk_entry_set_text(entry2_axis_ts_irp6p, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[2]);
					gtk_entry_set_text(entry3_axis_ts_irp6p, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[3]);
					gtk_entry_set_text(entry4_axis_ts_irp6p, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[4]);
					gtk_entry_set_text(entry5_axis_ts_irp6p, buf);
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[5]);
					gtk_entry_set_text(entry6_axis_ts_irp6p, buf);
					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry7_axis_ts_irp6p, buf);
				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_set_button_clicked_postument_axis_ts (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_axis_ts_irp6p"));
 		GtkSpinButton * spin2_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_axis_ts_irp6p"));
 		GtkSpinButton * spin3_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_axis_ts_irp6p"));
 		GtkSpinButton * spin4_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_axis_ts_irp6p"));
 		GtkSpinButton * spin5_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_axis_ts_irp6p"));
 		GtkSpinButton * spin6_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_axis_ts_irp6p"));
 		GtkSpinButton * spin7_axis_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_axis_ts_irp6p"));
 	    

		if (state_postument.is_synchronised)
		{
				tool_vector_a[0] = gtk_spin_button_get_value(spin1_axis_ts_irp6p);
				tool_vector_a[1] = gtk_spin_button_get_value(spin2_axis_ts_irp6p);
				tool_vector_a[2] = gtk_spin_button_get_value(spin3_axis_ts_irp6p);
				tool_vector_a[3] = gtk_spin_button_get_value(spin4_axis_ts_irp6p);
				tool_vector_a[4] = gtk_spin_button_get_value(spin5_axis_ts_irp6p);
				tool_vector_a[5] = gtk_spin_button_get_value(spin6_axis_ts_irp6p);
				tool_vector_a[6] = gtk_spin_button_get_value(spin7_axis_ts_irp6p);
	    
 		
 		wl = sqrt(tool_vector_a[3]*tool_vector_a[3] + tool_vector_a[4]*tool_vector_a[4] + tool_vector_a[5]*tool_vector_a[5]);

		if((wl > 1 + l_eps) || (wl < 1 - l_eps))
		{
			tool_vector_a[3] = tool_vector_a[3]/wl;
			tool_vector_a[4] = tool_vector_a[4]/wl;
			tool_vector_a[5] = tool_vector_a[5]/wl;
		}
		
		for(int i=3; i<7; i++)
		{
				tool_vector_a[i] *= tool_vector_a[6];
		}
		
			robot_postument->set_tool_xyz_angle_axis(tool_vector_a);		
		}
		on_read_button_clicked_postument_axis_ts (button, userdata);

	}

}


extern "C"
{
	void on_arrow_button_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_euler_xyz_irp6p"));
        GtkSpinButton * spin1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin1_euler_xyz_irp6p, atof(gtk_entry_get_text(entry1_euler_xyz_irp6p)));
	
        GtkEntry * entry2_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_euler_xyz_irp6p"));
        GtkSpinButton * spin2_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin2_euler_xyz_irp6p, atof(gtk_entry_get_text(entry2_euler_xyz_irp6p)));
	
        GtkEntry * entry3_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_euler_xyz_irp6p"));
        GtkSpinButton * spin3_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin3_euler_xyz_irp6p, atof(gtk_entry_get_text(entry3_euler_xyz_irp6p)));
	
        GtkEntry * entry4_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_euler_xyz_irp6p"));
        GtkSpinButton * spin4_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin4_euler_xyz_irp6p, atof(gtk_entry_get_text(entry4_euler_xyz_irp6p)));
	
        GtkEntry * entry5_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_euler_xyz_irp6p"));
        GtkSpinButton * spin5_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin5_euler_xyz_irp6p, atof(gtk_entry_get_text(entry5_euler_xyz_irp6p)));
	
        GtkEntry * entry6_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_euler_xyz_irp6p"));
        GtkSpinButton * spin6_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin6_euler_xyz_irp6p, atof(gtk_entry_get_text(entry6_euler_xyz_irp6p)));
	
        GtkEntry * entry7_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_euler_xyz_irp6p"));
        GtkSpinButton * spin7_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin7_euler_xyz_irp6p, atof(gtk_entry_get_text(entry7_euler_xyz_irp6p)));
	
	}
	
	void on_read_button_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_euler_xyz_irp6p"));
		GtkEntry * entry2_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_euler_xyz_irp6p"));
		GtkEntry * entry3_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_euler_xyz_irp6p"));
		GtkEntry * entry4_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_euler_xyz_irp6p"));
		GtkEntry * entry5_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_euler_xyz_irp6p"));
		GtkEntry * entry6_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_euler_xyz_irp6p"));
		GtkEntry * entry7_euler_xyz_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry7_euler_xyz_irp6p"));
	
 		
		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_postument->read_xyz_euler_zyz(irp6p_current_pos_e))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_e[0]);
					gtk_entry_set_text(entry1_euler_xyz_irp6p, buf);
					irp6p_desired_pos_e[0] = irp6p_current_pos_e[0];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_e[1]);
					gtk_entry_set_text(entry2_euler_xyz_irp6p, buf);
					irp6p_desired_pos_e[1] = irp6p_current_pos_e[1];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_e[2]);
					gtk_entry_set_text(entry3_euler_xyz_irp6p, buf);
					irp6p_desired_pos_e[2] = irp6p_current_pos_e[2];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_e[3]);
					gtk_entry_set_text(entry4_euler_xyz_irp6p, buf);
					irp6p_desired_pos_e[3] = irp6p_current_pos_e[3];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_e[4]);
					gtk_entry_set_text(entry5_euler_xyz_irp6p, buf);
					irp6p_desired_pos_e[4] = irp6p_current_pos_e[4];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_e[5]);
					gtk_entry_set_text(entry6_euler_xyz_irp6p, buf);
					irp6p_desired_pos_e[5] = irp6p_current_pos_e[5];				
					snprintf (buf, sizeof(buf), "%.3f", irp6p_current_pos_e[6]);
					gtk_entry_set_text(entry7_euler_xyz_irp6p, buf);
					irp6p_desired_pos_e[6] = irp6p_current_pos_e[6];				
		
 				
				for (int i = 0; i < 7; i++)
				irp6p_desired_pos_e[i] = irp6p_current_pos_e[i];		
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_xyz_irp6p"));
 		GtkSpinButton * spin2_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_xyz_irp6p"));
 		GtkSpinButton * spin3_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_xyz_irp6p"));
 		GtkSpinButton * spin4_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_xyz_irp6p"));
 		GtkSpinButton * spin5_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_xyz_irp6p"));
 		GtkSpinButton * spin6_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_xyz_irp6p"));
 		GtkSpinButton * spin7_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_euler_xyz_irp6p"));
 	    

		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) {
				irp6p_desired_pos_e[0] = gtk_spin_button_get_value(spin1_euler_xyz_irp6p);
				irp6p_desired_pos_e[1] = gtk_spin_button_get_value(spin2_euler_xyz_irp6p);
				irp6p_desired_pos_e[2] = gtk_spin_button_get_value(spin3_euler_xyz_irp6p);
				irp6p_desired_pos_e[3] = gtk_spin_button_get_value(spin4_euler_xyz_irp6p);
				irp6p_desired_pos_e[4] = gtk_spin_button_get_value(spin5_euler_xyz_irp6p);
				irp6p_desired_pos_e[5] = gtk_spin_button_get_value(spin6_euler_xyz_irp6p);
				irp6p_desired_pos_e[6] = gtk_spin_button_get_value(spin7_euler_xyz_irp6p);
	    
			
			robot_postument->move_xyz_euler_zyz(irp6p_desired_pos_e);
			}
			 if (state_postument.is_synchronised) {
				gtk_spin_button_set_value(spin1_euler_xyz_irp6p, irp6p_desired_pos_e[0]);
				gtk_spin_button_set_value(spin2_euler_xyz_irp6p, irp6p_desired_pos_e[1]);
				gtk_spin_button_set_value(spin3_euler_xyz_irp6p, irp6p_desired_pos_e[2]);
				gtk_spin_button_set_value(spin4_euler_xyz_irp6p, irp6p_desired_pos_e[3]);
				gtk_spin_button_set_value(spin5_euler_xyz_irp6p, irp6p_desired_pos_e[4]);
				gtk_spin_button_set_value(spin6_euler_xyz_irp6p, irp6p_desired_pos_e[5]);
				gtk_spin_button_set_value(spin7_euler_xyz_irp6p, irp6p_desired_pos_e[6]);
	  
			 }
		}
		on_read_button_clicked_postument_euler_xyz (button, userdata);

	}
	
	void on_export_button_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_xyz_irp6p"));
 		GtkSpinButton * spin2_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_xyz_irp6p"));
 		GtkSpinButton * spin3_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_xyz_irp6p"));
 		GtkSpinButton * spin4_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_xyz_irp6p"));
 		GtkSpinButton * spin5_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_xyz_irp6p"));
 		GtkSpinButton * spin6_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_xyz_irp6p"));
 		GtkSpinButton * spin7_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_euler_xyz_irp6p"));
 	
 		sprintf(buffer, "edp_irp6p XYZ EULER ZYZ position  %f %f %f %f %f %f %f" 
 		, gtk_spin_button_get_value(spin1_euler_xyz_irp6p), gtk_spin_button_get_value(spin2_euler_xyz_irp6p), gtk_spin_button_get_value(spin3_euler_xyz_irp6p), gtk_spin_button_get_value(spin4_euler_xyz_irp6p), gtk_spin_button_get_value(spin5_euler_xyz_irp6p), gtk_spin_button_get_value(spin6_euler_xyz_irp6p), gtk_spin_button_get_value(spin7_euler_xyz_irp6p));
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
		
		
 	    GtkSpinButton * spin1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin1_euler_xyz_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin2_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin2_euler_xyz_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin3_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin3_euler_xyz_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin4_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin4_euler_xyz_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin5_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin5_euler_xyz_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin6_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin6_euler_xyz_irp6p, atof(gtk_entry_get_text(entryConsole)));
	
 	    GtkSpinButton * spin7_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin7_euler_xyz_irp6p, atof(gtk_entry_get_text(entryConsole)));
	  
 	}
	

	void on_button1_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin1_euler_xyz_irp6p, gtk_spin_button_get_value(spin1_euler_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	} 
	
	void on_button2_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin1_euler_xyz_irp6p, gtk_spin_button_get_value(spin1_euler_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	}   

	void on_button3_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin2_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin2_euler_xyz_irp6p, gtk_spin_button_get_value(spin2_euler_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	} 
	
	void on_button4_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin2_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin2_euler_xyz_irp6p, gtk_spin_button_get_value(spin2_euler_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	}   

	void on_button5_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin3_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin3_euler_xyz_irp6p, gtk_spin_button_get_value(spin3_euler_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	} 
	
	void on_button6_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin3_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin3_euler_xyz_irp6p, gtk_spin_button_get_value(spin3_euler_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	}   

	void on_button7_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin4_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin4_euler_xyz_irp6p, gtk_spin_button_get_value(spin4_euler_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	} 
	
	void on_button8_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin4_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin4_euler_xyz_irp6p, gtk_spin_button_get_value(spin4_euler_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	}   

	void on_button9_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin5_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin5_euler_xyz_irp6p, gtk_spin_button_get_value(spin5_euler_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	} 
	
	void on_button10_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin5_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin5_euler_xyz_irp6p, gtk_spin_button_get_value(spin5_euler_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	}   

	void on_button11_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin6_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin6_euler_xyz_irp6p, gtk_spin_button_get_value(spin6_euler_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	} 
	
	void on_button12_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin6_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin6_euler_xyz_irp6p, gtk_spin_button_get_value(spin6_euler_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	}   

	void on_button13_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin7_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin7_euler_xyz_irp6p, gtk_spin_button_get_value(spin7_euler_xyz_irp6p) - gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	} 
	
	void on_button14_clicked_postument_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbuttonDown1_euler_xyz_irp6p"));
        GtkSpinButton * spin7_euler_xyz_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton7_euler_xyz_irp6p"));
        gtk_spin_button_set_value(spin7_euler_xyz_irp6p, gtk_spin_button_get_value(spin7_euler_xyz_irp6p) + gtk_spin_button_get_value(spinbuttonDown1_euler_xyz_irp6p));
 		
 		on_execute_button_clicked_postument_euler_xyz (button, userdata);
 	}   

}


extern "C"
{
	void on_arrow_button_clicked_postument_euler_ts (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		
        GtkEntry * entry1_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_euler_ts_irp6p"));
        GtkSpinButton * spin1_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_ts_irp6p"));
        gtk_spin_button_set_value(spin1_euler_ts_irp6p, atof(gtk_entry_get_text(entry1_euler_ts_irp6p)));
	
        GtkEntry * entry2_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_euler_ts_irp6p"));
        GtkSpinButton * spin2_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_ts_irp6p"));
        gtk_spin_button_set_value(spin2_euler_ts_irp6p, atof(gtk_entry_get_text(entry2_euler_ts_irp6p)));
	
        GtkEntry * entry3_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_euler_ts_irp6p"));
        GtkSpinButton * spin3_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_ts_irp6p"));
        gtk_spin_button_set_value(spin3_euler_ts_irp6p, atof(gtk_entry_get_text(entry3_euler_ts_irp6p)));
	
        GtkEntry * entry4_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_euler_ts_irp6p"));
        GtkSpinButton * spin4_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_ts_irp6p"));
        gtk_spin_button_set_value(spin4_euler_ts_irp6p, atof(gtk_entry_get_text(entry4_euler_ts_irp6p)));
	
        GtkEntry * entry5_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_euler_ts_irp6p"));
        GtkSpinButton * spin5_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_ts_irp6p"));
        gtk_spin_button_set_value(spin5_euler_ts_irp6p, atof(gtk_entry_get_text(entry5_euler_ts_irp6p)));
	
        GtkEntry * entry6_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_euler_ts_irp6p"));
        GtkSpinButton * spin6_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_ts_irp6p"));
        gtk_spin_button_set_value(spin6_euler_ts_irp6p, atof(gtk_entry_get_text(entry6_euler_ts_irp6p)));
		
 	}
	
	void on_read_button_clicked_postument_euler_ts (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkEntry * entry1_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry1_euler_ts_irp6p"));
		GtkEntry * entry2_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry2_euler_ts_irp6p"));
		GtkEntry * entry3_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry3_euler_ts_irp6p"));
		GtkEntry * entry4_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry4_euler_ts_irp6p"));
		GtkEntry * entry5_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry5_euler_ts_irp6p"));
		GtkEntry * entry6_euler_ts_irp6p = GTK_ENTRY(gtk_builder_get_object(&thisBuilder, "entry6_euler_ts_irp6p"));
	
 		
		if (robot_postument->ecp->get_EDP_pid()!=-1)
		{
			if (state_postument.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_postument->read_tool_xyz_euler_zyz(tool_vector_e))) // Odczyt polozenia walow silnikow
					printf("Blad w read external\n");
					
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[0]);
					gtk_entry_set_text(entry1_euler_ts_irp6p, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[1]);
					gtk_entry_set_text(entry2_euler_ts_irp6p, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[2]);
					gtk_entry_set_text(entry3_euler_ts_irp6p, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[3]);
					gtk_entry_set_text(entry4_euler_ts_irp6p, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[4]);
					gtk_entry_set_text(entry5_euler_ts_irp6p, buf);		
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_e[5]);
					gtk_entry_set_text(entry6_euler_ts_irp6p, buf);		
				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout << "Robot is not synchronized" << std::endl;
			}
		}
	
	}
	
	void on_set_button_clicked_postument_euler_ts (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder & thisBuilder = ((*ChoseEntry).getBuilder());
        
		GtkSpinButton * spin1_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton1_euler_ts_irp6p"));
 		GtkSpinButton * spin2_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton2_euler_ts_irp6p"));
 		GtkSpinButton * spin3_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton3_euler_ts_irp6p"));
 		GtkSpinButton * spin4_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton4_euler_ts_irp6p"));
 		GtkSpinButton * spin5_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton5_euler_ts_irp6p"));
 		GtkSpinButton * spin6_euler_ts_irp6p = GTK_SPIN_BUTTON(gtk_builder_get_object(&thisBuilder, "spinbutton6_euler_ts_irp6p"));
 	    

		if (state_postument.is_synchronised)
		{
				tool_vector_e[0] = gtk_spin_button_get_value(spin1_euler_ts_irp6p);
				tool_vector_e[1] = gtk_spin_button_get_value(spin2_euler_ts_irp6p);
				tool_vector_e[2] = gtk_spin_button_get_value(spin3_euler_ts_irp6p);
				tool_vector_e[3] = gtk_spin_button_get_value(spin4_euler_ts_irp6p);
				tool_vector_e[4] = gtk_spin_button_get_value(spin5_euler_ts_irp6p);
				tool_vector_e[5] = gtk_spin_button_get_value(spin6_euler_ts_irp6p);
	    
			
			robot_postument->set_tool_xyz_euler_zyz(tool_vector_e);
		}
		on_read_button_clicked_postument_euler_ts (button, userdata);

	}
}
