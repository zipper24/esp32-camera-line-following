idf_component_register(
    SRCS "red_ball_left_goal_main.c" "tjpgd.c" "teal_ball_color.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_timer esp_event esp_netif esp_wifi esp_http_server nvs_flash
)
