// rustでros2を使えるようにする
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info,
};

fn main() -> Result<(), DynError> {
    // コンテキストの作成
    let ctx = Context::new()?;

    // ノードを作成
    let node = ctx.create_node("my_listener", None, Default::default())?;

    // subscriberの作成
    let subscriber = node.create_subscriber::<std_msgs::msg::String>("my_topic", None, true)?;

    // ロガーの作成
    let logger = Logger::new("my_listener");

    // セレクターの作成
    let mut selector = ctx.create_selector()?;

    // サブスクライバー用callbackを作成
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            // サブスクライブしたデータを表示
            pr_info!(logger, "receive: {}", msg.data);
        }),
    );

    // 無限ループ
    loop {
        selector.wait()?;
    }
}