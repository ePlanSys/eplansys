# The plugin is loaded by name here as the executor loads it, which needs the
# class to be findable through the ament index — so the test links the library
# rather than the plugin, and pluginlib finds the installed one.
ament_add_gtest(epistemic_bt_builder_test epistemic_bt_builder_test.cpp TIMEOUT 120)
target_link_libraries(epistemic_bt_builder_test
  ${PROJECT_NAME}
  behaviortree_cpp::behaviortree_cpp
  pluginlib::pluginlib
)
