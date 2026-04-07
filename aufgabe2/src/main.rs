use std::fs::read_to_string;
use std::path::Path;

#[derive(Debug)]
struct Tree {
    id: u32,
    x: u32,
    y: u32
}

macro_rules! scan {
    ($it:expr, $($t:ty),*) => {
        ($( $it.next().expect("Unexpected EOF").parse::<$t>().expect("Parse error") ),*)
    };
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let in_path = args.get(1).map(|s| s.as_str()).unwrap_or("resources/roboter01.txt");
    let out_path = if args.len() > 2 {
        args[2].clone()
    } else {
        let path = Path::new(in_path);
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("out");
        format!("{}_out.txt", stem)
    };

    let data = read_to_string(in_path).expect("Failed to read file");
    let mut it = data.split_whitespace();

    let max_time = scan!(it, u32);
    let tree_count = scan!(it, u32);
    let mut trees = Vec::new();
    
    for _ in 0..tree_count {
        let tree = scan!(it, u32, u32, u32);
        trees.push(Tree { id: tree.0, x: tree.1, y: tree.2 });
    }
    
    println!("{:?}", trees)
}
