enum _RelayState {
  RELAY_OFF,
  RELAY_ON,
  RELAY_WAITING  /* we're off duty in our cycle */
};

enum _MenuType {
  STATUS,
  NUMBER,
  ONOFF,
  ROLE,
  TEXT,
  SOIL_CALIBRATE,
  UPTIME,
  TIME,
  SUBMENU,
  LANGUAGE,
  LIGHTS_RESET,
  STORE_SETTINGS,
  RESET_SETTINGS,
  LOAD_SETTINGS,
  BACK,
  LOG
};

struct _Relay {
  int gpio_pin;
  int role;
  int state;
};

struct _ConfigItem {
  void  *data;   /* pointer to memory variable for menu item */
  int    type;   /* what type of entry this is */
  int    eeprom_pos; /* storage slot in arduino EEPROM memory */
  float  default_value;
};

struct _MenuItem {
  int    label;  /* first text line */
  int    label2; /* second text line */
  int    type;   /* what type of menu is this */
  void  *data;   /* pointer to memory variable for menu item */
};

typedef enum _RelayState RelayState;
typedef struct _Relay Relay;
typedef enum _MenuType MenuType;
typedef struct _MenuItem MenuItem;
typedef struct _ConfigItem ConfigItem;
