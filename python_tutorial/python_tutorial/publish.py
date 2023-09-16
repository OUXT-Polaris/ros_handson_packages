# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# こちらのサンプルコードはこちら（https://github.com/ros2/examples/blob/rolling/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py）からライセンスを確認の上拝借しています。

# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。
import rclpy
# rclpy (ROS 2のpythonクライアント)の機能のうちNodeを簡単に使えるようにします。こう書いていない場合、Nodeではなくrclpy.node.Nodeと書く必要があります。
from rclpy.node import Node

# ROS 2の文字列型を使えるようにimport
from std_msgs.msg import String

# C++と同じく、Node型を継承します。
class MinimalPublisher(Node):
    # コンストラクタです、MinimulPublisherクラスのインスタンスを作成する際に呼び出されます。
    def __init__(self):
        # 継承元のクラスを初期化します。（https://www.python-izm.com/advanced/class_extend/）今回の場合継承するクラスはNodeになります。
        super().__init__('minimal_publisher')
        # publisherを作成します。self.と頭についていますが、これはself.publisherがメンバー変数であることを表しています。
        self.publisher_ = self.create_publisher(
            # String型のデータを受信することを示します。
            String, 
            # topicという名前のtopicにデータを送信します。
            'topic',
            # MessageのBuffer Sizeです。基本的には1で問題ありません。
            10)
        timer_period = 0.5  # seconds
        # タイマーを作成、一定時間ごとにコールバックを実行できるように設定
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # タイマーによって実行されるコールバック
    def timer_callback(self):
        # 文字列型のインスタンスを作成します、この変数はローカル変数のため、コールバックの外からはアクセスできません。
        msg = String()
        # Stringのメッセージ型の変数に文字列を代入
        msg.data = 'Hello World: %d' % self.i
        # Publisher経由でメッセージを発行
        self.publisher_.publish(msg)
        # Loggerでterminalにログを出力
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.iというメンバ変数をカウントアップする。
        self.i += 1


# mainという名前の関数です。C++のmain関数とは異なり、これは処理の開始地点ではありません。
def main(args=None):
    # rclpyの初期化処理です。ノードを立ち上げる前に実装する必要があります。
    rclpy.init(args=args)
    # MinimalPublisherクラスのインスタンスを作成
    minimal_publisher = MinimalPublisher()
    # spin処理を実行、spinをしていないとROS 2のノードはデータを入出力することが出来ません。
    rclpy.spin(minimal_publisher)
    # 明示的にノードの終了処理を行います。
    minimal_publisher.destroy_node()
    # rclpyの終了処理、これがないと適切にノードが破棄されないため様々な不具合が起こります。
    rclpy.shutdown()


# 本スクリプト(publish.py)の処理の開始地点です。
if __name__ == '__main__':
    # 関数`main`を実行する。
    main()