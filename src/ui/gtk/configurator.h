#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <gtk/gtktreestore.h>
#include <unistd.h>

#include <iostream>

#ifndef __CONFIGURATOR_H

class configurator {
private:
	// XML config document
	xmlDocPtr doc;

	// GtkTreeView model
	GtkTreeStore *store;

	// configuration files directory
	char config_dir[PATH_MAX];

	xmlXPathObjectPtr getnodeset (xmlDocPtr doc, const xmlChar *xpath);

	int populate_tree_model();

public:

	enum TREE_VIEW_COLUMNS
	{
		NAME_COLUMN, /* name */
		NODE_NAME_COLUMN, /* Executing node of the process */
		IS_RUNNING_COLUMN, /* Is currently running? */
		N_COLUMNS
	};

	enum TREE_VIEW_MAIN_ROWS {
		MASTER_PROCESS_MAIN_ROW,
		SENSORS_MAIN_ROW,
		EFFECTORS_MAIN_ROW,
		N_ROWS
	};

	configurator();
	~configurator();

	int open_config_file(const char *filename);

	std::string get_string(const xmlChar *xpath);

    GtkTreeStore *getStore() const
    {
        return store;
    }

    const char *getConfig_dir() const
    {
        return config_dir;
    }

};

extern class configurator *config;

#endif
