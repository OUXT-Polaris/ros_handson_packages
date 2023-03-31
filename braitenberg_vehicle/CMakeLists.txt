cmake_minimum_required(VERSION 3.8)
project(braitenberg_vehicle)

# コンパイラに引き渡すコンパイルオプションを設定
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()

# 共有ライブラリ「braitenberg_vehicle_controller」をコンパイル、その際依存しているパッケージに含まれるライブラリ等を全てリンクする
# 共有ライブラリ「braitenberg_vehicle_controller」はsrc/braitenberg_vehicle_controller.cpp、src/motion_model.cppがコンパイルされる
ament_auto_add_library(braitenberg_vehicle_controller
  src/braitenberg_vehicle_controller.cpp
  src/motion_model.cpp)
# 共有ライブラリ「braitenberg_vehicle_controller 」にある
#「braitenberg_vehicle::BraitenbergVehicleController」クラスはコンポーネントであることをcolconに伝える
rclcpp_components_register_nodes(braitenberg_vehicle_controller 
  "braitenberg_vehicle::BraitenbergVehicleController")

# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# テスト時の設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # package.xmlのtest_dependを確認し自動で必要なテストを実行する
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
