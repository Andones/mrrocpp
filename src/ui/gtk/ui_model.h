#ifndef __UI_MODEL_H
#define __UI_MODEL_H

#include <gtk/gtk.h>
#include <gtkmm.h>

#include "ui_config_entry.h"

#include "lib/configurator.h"
#include "lib/srlib.h"

class ui_model
{
	public:
		static ui_model& instance();

		void clear(void);

		ui_config_entry & add_ui_config_entry(ui_config_entry & parent, ui_config_entry::ui_config_entry_type entry_type, const char *program_name, const char *node_name = NULL, const char *ui_def = NULL);

		GtkTreeStore * getStore() const
		{
			return store;
		}

		ui_config_entry & getRootNode()
		{
			return root_entry;
		}

		ui_config_entry & getNodeByPath(GtkTreePath *path);

		enum TREE_VIEW_COLUMNS
		{
			NAME_COLUMN, /* name */
			NODE_NAME_COLUMN, /* Executing node of the process */
			IS_RUNNING_COLUMN, /* Is currently running? */
			N_COLUMNS
		};

		enum TREE_VIEW_MAIN_ROWS
		{
			MASTER_PROCESS_MAIN_ROW, SENSORS_MAIN_ROW, EFFECTORS_MAIN_ROW, N_ROWS
		};

		//! method to manage tab panel visibility
		void show_page(bool visible);

		//! set properties of buttons
		void setMpLoadButton (bool sensitive, bool button_type_is_load);
		void setEdpsLoadButton (bool sensitive, bool button_type_is_load);

		//! load EDP processes
		void loadEdps(void);

		//! set status bar message
		guint set_status(const char *msg);

		//! set current notebook page
		void set_current_page(gint page_num);

		GObject *getUiGObject(const gchar *name);
		Glib::RefPtr<Glib::Object> getUiObject(const gchar *name);

		//! initialize SR client objects
		void init_sr(void);

	private:
		ui_model();

		ui_config_entry root_entry;

		// GtkTreeView model
		GtkTreeStore *store;

		ui_model(ui_model const&); //not defined, not copyable
		ui_model& operator=(ui_model const&); //not defined, not assignable
		~ui_model();

		int tabs_visible;

		GtkBuilder *builder;

		int set_tree_view(void);

		//! old-type .INI configurator
		configurator *config;

		//! SR object for UI
		sr_ui* ui_report;

		//! SR object for UI
		sr_ecp* ecp_report;
};

/*
namespace {
struct ForceSingletonInitialization
{
	ForceSingletonInitialization()
	{
		printf("qpa1\n");
		ui_model::instance();
		printf("qpa2\n");
	}
} instance;
}
 */

#endif /* __UI_MODEL_H */
