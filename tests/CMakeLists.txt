include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# 编译测试可执行文件
add_executable(ukf_test ukf_test.cpp)
target_link_libraries(ukf_test gtest_main ukf_lib)

# 添加测试
add_test(NAME ukf_test COMMAND ukf_test)
