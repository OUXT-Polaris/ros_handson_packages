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

# こちらのサンプルコードはこちら（https://github.com/ros2/examples/blob/rolling/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py）からライセンスを確認の上拝借しています。

# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。
import rclpy
# rclpy (ROS 2のpythonクライアント)の機能のうちNodeを簡単に使えるようにします。こう書いていない場合、Nodeではなくrclpy.node.Nodeと書く必要があります。
from rclpy.node import Node

# ROS 2の文字列型を使えるようにimport
from std_msgs.msg import String


# C++と同じく、Node型を継承します。
class MinimalSubscriber(Node):
    # コンストラクタです、MinimalSubscriberクラスのインスタンスを作成する際に呼び出されます。
    def __init__(self):
        # 継承元のクラスを初期化します。（https://www.python-izm.com/advanced/class_extend/）今回の場合継承するクラスはNodeになります。
        super().__init__('minimal_subscriber')
        # Subscriptionを作成、Publishしたデータを受け取ることが出来るようにします。
        self.subscription = self.create_subscription(
            # String型を
            String,
            # topicという名前のtopicをsubscribeします。
            'topic',
            # topicという名前のtopicに新しいデータが来たときにlistener_callbackを実行します。selfがついているのはlistener_callbackがメンバー変数であることを示しています。
            self.listener_callback,
            # メッセージの受信キューのサイズ
            10)
        self.subscription  # 警告を回避するために設置されているだけです。削除しても挙動はかわりません。

    # メッセージを受け取ったときに実行されるコールバック関数
    def listener_callback(self, msg):
        # loggerを使ってterminalに文字列を出力する。
        self.get_logger().info('I heard: "%s"' % msg.data)


# mainという名前の関数です。C++のmain関数とは異なり、これは処理の開始地点ではありません。
def main(args=None):
    # rclpyの初期化処理です。ノードを立ち上げる前に実装する必要があります。
    rclpy.init(args=args)
    # MinimalSubscriberクラスのインスタンスを作成
    minimal_subscriber = MinimalSubscriber()
    # spin処理を実行、spinをしていないとROS 2のノードはデータを入出力することが出来ません。
    rclpy.spin(minimal_subscriber)
    # 明示的にノードの終了処理を行います。
    minimal_subscriber.destroy_node()
    # rclpyの終了処理、これがないと適切にノードが破棄されないため様々な不具合が起こります。
    rclpy.shutdown()


# 本スクリプト(publish.py)の処理の開始地点です。
if __name__ == '__main__':
    # 関数`main`を実行する。
    main()
