FILE(REMOVE_RECURSE
  "../src/morse_jido_manipulation_action/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/morse_jido_manipulation_action/msg/__init__.py"
  "../src/morse_jido_manipulation_action/msg/_arm_manipulationAction.py"
  "../src/morse_jido_manipulation_action/msg/_arm_manipulationGoal.py"
  "../src/morse_jido_manipulation_action/msg/_arm_manipulationActionGoal.py"
  "../src/morse_jido_manipulation_action/msg/_arm_manipulationResult.py"
  "../src/morse_jido_manipulation_action/msg/_arm_manipulationActionResult.py"
  "../src/morse_jido_manipulation_action/msg/_arm_manipulationFeedback.py"
  "../src/morse_jido_manipulation_action/msg/_arm_manipulationActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
