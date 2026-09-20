use clap::Parser;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use talos_ipc::{GimbalCmd, GimbalCmdPublisher};

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "Talos 云台命令发送工具：向运行中的仿真器发送 GimbalCmd 角度差（需先启动仿真器、按 F5）"
)]
struct Args {
    /// 每条消息的 yaw 增量（度，俯视逆时针为正）
    #[arg(long, default_value_t = 0.0)]
    yaw_diff: f32,
    /// 每条消息的 pitch 增量（度，talos 链路符号约定：正值 = 低头，与 ROS2 链路相反）
    #[arg(long, default_value_t = 0.0)]
    pitch_diff: f32,
    /// 目标距离（米）；-1 = 放弃目标
    #[arg(long, default_value_t = 3.0)]
    distance: f32,
    /// 触发一次开火（仿真器内限频 10Hz）
    #[arg(long, default_value_t = false)]
    fire: bool,
    /// 发送频率 Hz（仅 count > 1 时有意义）
    #[arg(long, default_value_t = 10.0)]
    hz: f32,
    /// 发送条数；0 = 一直发到 Ctrl+C。增量语义下默认单发一条最有用
    #[arg(long, default_value_t = 1)]
    count: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let args = Args::parse();

    let mut publisher = GimbalCmdPublisher::connect()?;
    println!(
        "connected to talos shm: yaw_diff={}° pitch_diff={}° distance={} fire={} hz={} count={}",
        args.yaw_diff,
        args.pitch_diff,
        args.distance,
        args.fire,
        args.hz,
        if args.count == 0 {
            "∞".to_string()
        } else {
            args.count.to_string()
        }
    );

    let interval = Duration::from_secs_f32(1.0 / args.hz.max(0.1));
    let mut published: u32 = 0;
    let mut next_deadline = Instant::now();

    loop {
        if args.count != 0 && published >= args.count {
            break;
        }

        let cmd = GimbalCmd {
            timestamp_ns: now_ns(),
            yaw_diff_deg: args.yaw_diff,
            pitch_diff_deg: args.pitch_diff,
            distance_m: args.distance,
            fire_advice: u8::from(args.fire),
            ..GimbalCmd::default()
        };
        // 背压：上一条未被消费就原地重试同一条命令，增量绝不因覆写丢失。
        if publisher.try_publish(cmd) {
            published += 1;
            println!(
                "sent #{published}: yaw_diff={:.2}° pitch_diff={:.2}° distance={:.2} fire={}",
                args.yaw_diff, args.pitch_diff, args.distance, args.fire
            );
            next_deadline += interval;
            let wait = next_deadline.saturating_duration_since(Instant::now());
            thread::sleep(wait.max(Duration::from_millis(1)));
        } else {
            thread::sleep(Duration::from_millis(1));
        }
    }

    println!("done, total sent={published}");
    Ok(())
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}
