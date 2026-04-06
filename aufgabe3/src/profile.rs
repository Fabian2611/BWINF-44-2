#[macro_export]
macro_rules! profile {
    ($name:ident) => {{
        use std::time::Instant;
        use sysinfo::{System};

        let mut sys = System::new();
        sys.refresh_processes(sysinfo::ProcessesToUpdate::All, true);

        let pid = sysinfo::get_current_pid().unwrap();

        let before_mem = sys.process(pid)
            .map(|p| p.memory())
            .unwrap_or(0);

        let start = Instant::now();
        $name();
        let duration = start.elapsed();

        sys.refresh_processes(sysinfo::ProcessesToUpdate::All, true);

        let after_mem = sys.process(pid)
            .map(|p| p.memory())
            .unwrap_or(0);

        println!(
            "{} took {:?} | {} KB RAM",
            stringify!($name),
            duration,
            after_mem as i64 - before_mem as i64
        );
    }};
}
