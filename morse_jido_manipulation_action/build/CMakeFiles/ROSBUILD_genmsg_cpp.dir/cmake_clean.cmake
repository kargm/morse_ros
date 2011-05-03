FILE(REMOVE_RECURSE
  "../src/morse_jido_manipulation_action/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/morse_jido_manipulation_action/arm_manipulationAction.h"
  "../msg_gen/cpp/include/morse_jido_manipulation_action/arm_manipulationGoal.h"
  "../msg_gen/cpp/include/morse_jido_manipulation_action/arm_manipulationActionGoal.h"
  "../msg_gen/cpp/include/morse_jido_manipulation_action/arm_manipulationResult.h"
  "../msg_gen/cpp/include/morse_jido_manipulation_action/arm_manipulationActionResult.h"
  "../msg_gen/cpp/include/morse_jido_manipulation_action/arm_manipulationFeedback.h"
  "../msg_gen/cpp/include/morse_jido_manipulation_action/arm_manipulationActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
