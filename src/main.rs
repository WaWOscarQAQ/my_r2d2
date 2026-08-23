use my_r2d2::utils::yaml_reader::YamlEnv;
use std::path::Path;

fn usage() -> ! {
    eprintln!("usage: my_r2d2 yaml-get KEY");
    std::process::exit(2);
}

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(command) = args.next() else {
        println!("R2D2 core is ready");
        return;
    };
    if command != "yaml-get" {
        usage();
    }
    let Some(key) = args.next() else {
        usage();
    };
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let env = match YamlEnv::load(repo_root) {
        Ok(env) => env,
        Err(message) => {
            eprintln!("{message}");
            std::process::exit(1);
        }
    };
    match env.require_string(&key) {
        Ok(value) => print!("{value}"),
        Err(message) => {
            eprintln!("{message}");
            std::process::exit(1);
        }
    }
}
