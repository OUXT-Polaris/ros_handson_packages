cmake_minimum_required(VERSION 3.8)
# project名
project(tutorial)

# コンパイラに引き渡すコンパイルオプションを設定
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xmlのdependをチェックして自動的にfind_packageしてくれる
ament_auto_find_build_dependencies()

# 共有ライブラリ「subscribe」をコンパイル、その際依存しているパッケージに含まれるライブラリ等を全てリンクする
# 共有ライブラリ「subscribe」はsrc/subscribe.cppの中身よりコンパイルされる
ament_auto_add_library(subscribe src/subscribe.cpp)
# 共有ライブラリ「subscribe」にある「tutorial::Subscribe」クラスはコンポーネントであることをcolconに伝える
rclcpp_components_register_nodes(subscribe "tutorial::Subscribe")
# 共有ライブラリ「publish」をコンパイル、その際依存しているパッケージに含まれるライブラリ等を全てリンクする
# 共有ライブラリ「publish」はsrc/publish.cppの中身よりコンパイルされる
ament_auto_add_library(publish src/publish.cpp)
# 共有ライブラリ「publish」にある「tutorial::Publish」クラスはコンポーネントであることをcolconに伝える
rclcpp_components_register_nodes(publish "tutorial::Publish")
# 実行ファイル「pub_sub_node」をビルド、その際依存しているパッケージに含まれるライブラリ等を全てリンクする
ament_auto_add_executable(pub_sub_node
  src/pub_sub_node.cpp
  src/publish.cpp
  src/subscribe.cpp)

# launchファイルをインストール（ament_package_indexが探せる場所に置く）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # package.xmlのtest_dependを確認し自動で必要なテストを実行する
  ament_lint_auto_find_test_dependencies()
endif()

# ヘッダーのインストールや共有ライブラリのエクスポート等を自動で行ってくれる
ament_auto_package()
