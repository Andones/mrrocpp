#if !defined(_CONFIG_TYPES_H)
#define _CONFIG_TYPES_H

#define CONFIG_MAX_KEY_LEN				80
#define CONFIG_MAX_SECTION_NAME_LEN		80
#define CONFIG_MAX_CONFIGFILE_LEN		80
#define CONFIG_MAX_NODENAME_LEN			20

#define CONFIGSRV_CHANNEL_NAME			"configsrv"

typedef enum _config_request {
	CONFIG_CHANGE_INI_FILE,
	CONFIG_RETURN_INT_VALUE,
	CONFIG_RETURN_DOUBLE_VALUE,
	CONFIG_RETURN_STRING_VALUE,
	CONFIG_EXISTS
} config_request_t;

typedef struct config_query {
	union data_t {
		struct query_t {
			char key[CONFIG_MAX_KEY_LEN];
			char section[CONFIG_MAX_SECTION_NAME_LEN];
		} query;
		char configfile[CONFIG_MAX_CONFIGFILE_LEN];
	} data;
} config_msg_t;

#endif /* _CONFIG_TYPES_H */
