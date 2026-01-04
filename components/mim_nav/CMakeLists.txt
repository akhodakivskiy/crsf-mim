idf_component_register(
  SRCS
      "mim_nav.c"
      "nav.c"
      "nav_pitcher.c"
  INCLUDE_DIRS "."
  REQUIRES
    la
    mim_conn
    mim_rc
    mim_settings
)
