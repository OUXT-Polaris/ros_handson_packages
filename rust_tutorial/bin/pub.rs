// rustでros2を使えるようにする
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info
};

// パラメータを取得可能にする
use ros2_rust_util::get_str_parameter;


// プログラムをsleepできるようにする
use std::time::Duration;

fn main() -> Result<(), DynError> {
    // コンテキストの作成
    let ctx = Context::new()?;

    // ノードを作成
    let node = ctx.create_node("my_talker", None, Default::default())?;

    // publisherの作成
    let publisher = node.create_publisher::<std_msgs::msg::String>("my_topic", None, true)?;

    // パラメータの取得 
    let param = get_str_parameter(node.get_name(), "name", "saito");


    // ロガーの作成
    let logger = Logger::new("my_talker");

    //カウンター変数
    let mut cnt = 0;

    // pub用メッセージ変数
    let mut msg = std_msgs::msg::String::new().unwrap();

    // 無限ループ
    loop {
        // pubするためのメッセージを作成
        let data = format!("Hello, World!: cnt = {cnt}");

        // Stringデータは直接代入するのではなくassignを使用する
        msg.data.assign(&data);

        // ターミナル出力
        pr_info!(logger, "{:?}-san, send: {}", param, msg.data);

        // publish!
        publisher.send(&msg)?;

        // カウント変数に追加
        cnt += 1;

        // 1秒 sleep
        std::thread::sleep(Duration::from_secs(1));
    }
}