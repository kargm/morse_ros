FILE(REMOVE_RECURSE
  "../src/morse_jido_manipulation_action/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/arm_manipulationAction.msg"
  "../msg/arm_manipulationGoal.msg"
  "../msg/arm_manipulationActionGoal.msg"
  "../msg/arm_manipulationResult.msg"
  "../msg/arm_manipulationActionResult.msg"
  "../msg/arm_manipulationFeedback.msg"
  "../msg/arm_manipulationActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
