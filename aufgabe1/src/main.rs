#![allow(dead_code)]

use std::io::Write;
mod profile;

use std::collections::HashMap;
use std::fs;
use std::fs::{File, OpenOptions};

#[derive(PartialEq, Eq, Debug, Copy, Clone)]
enum Direction {
    Up,
    Down,
    Left,
    Right,
}

#[derive(Debug, Clone)]
struct Edge {
    v1: u32,
    d1: Direction,
    v2: u32,
    d2: Direction,
}

#[derive(Debug, Clone)]
struct GraphR {
    n: u32,
    edges: Vec<Edge>,
}

/// (deg(v), deg_v(v), deg_h(v), sorted list of neighbor degrees)
type Signature = (u32, u32, u32, Vec<u32>);

impl GraphR {
    pub fn new(n: u32, edges: Vec<Edge>) -> Self {
        GraphR { n, edges }
    }

    /// Returns the degree of the specified vertex.
    pub fn degree(&self, vertex: u32) -> u32 {
        self.edges
            .iter()
            .filter(|e| e.v1 == vertex || e.v2 == vertex)
            .count() as u32
    }

    /// Returns the vertical degree of the specified vertex. Invariant when flipping.
    pub fn degree_vertical(&self, vertex: u32) -> u32 {
        self.edges
            .iter()
            .filter(|e| {
                (e.v1 == vertex && (e.d1 == Direction::Up || e.d1 == Direction::Down))
                    || (e.v2 == vertex && (e.d2 == Direction::Up || e.d2 == Direction::Down))
            })
            .count() as u32
    }

    /// Returns the horizontal degree of the specified vertex. Invariant when flipping.
    pub fn degree_horizontal(&self, vertex: u32) -> u32 {
        self.edges
            .iter()
            .filter(|e| {
                (e.v1 == vertex && (e.d1 == Direction::Left || e.d1 == Direction::Right))
                    || (e.v2 == vertex && (e.d2 == Direction::Left || e.d2 == Direction::Right))
            })
            .count() as u32
    }

    pub fn degree_sequence(&self) -> Vec<u32> {
        let mut degrees: Vec<u32> = (0..self.n).map(|v| self.degree(v)).collect();
        degrees.sort_unstable();
        degrees
    }

    /// Returns the signatures of all vertices, ordered.
    /// The signature of a vertex is a tuple of:
    /// (degree, vertical degree, horizontal degree, sorted list of neighbor degrees)
    /// As UP <-> DOWN and LEFT <-> RIGHT, the signature is invariant when flipping.
    pub fn signatures(&self) -> Vec<Signature> {
        let mut sigs = Vec::new();
        for v in 0..self.n {
            let deg = self.degree(v);
            let deg_v = self.degree_vertical(v);
            let deg_h = self.degree_horizontal(v);
            let mut neighbor_degs = self.neighbour_degrees(v);
            neighbor_degs.sort_unstable();
            sigs.push((deg, deg_v, deg_h, neighbor_degs));
        }
        sigs.sort_unstable();
        sigs
    }

    pub fn signature_to_node_list(&self) -> HashMap<Signature, Vec<u32>> {
        let mut sig_map: HashMap<Signature, Vec<u32>> = HashMap::new();
        for v in 0..self.n {
            let sig = (
                self.degree(v),
                self.degree_vertical(v),
                self.degree_horizontal(v),
                {
                    let mut neighbor_degrees = self.neighbour_degrees(v);
                    neighbor_degrees.sort_unstable();
                    neighbor_degrees
                },
            );
            sig_map.entry(sig).or_default().push(v);
        }
        sig_map
    }

    pub fn neighbour_degrees(&self, vertex: u32) -> Vec<u32> {
        let mut neighbor_degrees = Vec::new();
        for e in &self.edges {
            if e.v1 == vertex {
                neighbor_degrees.push(self.degree(e.v2));
            } else if e.v2 == vertex {
                neighbor_degrees.push(self.degree(e.v1));
            }
        }
        neighbor_degrees
    }
}

fn flip(dir: Direction) -> Direction {
    match dir {
        Direction::Up => Direction::Down,
        Direction::Down => Direction::Up,
        Direction::Left => Direction::Right,
        Direction::Right => Direction::Left,
    }
}

fn parse_graphs(
    path: &str,
) -> Result<Vec<(GraphR, Vec<Vec<char>>, HashMap<(usize, usize), u32>)>, String> {
    let inp = fs::read_to_string(path).map_err(|_| "Failed to read file".to_string())?;
    let mut lines = inp.lines().peekable();
    let mut all_graphs = Vec::new();

    while let Some(line) = lines.next() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }

        let size: Vec<u32> = trimmed
            .split_whitespace()
            .map(|s| {
                s.parse::<u32>()
                    .map_err(|_| "Invalid size values".to_string())
            })
            .collect::<Result<_, _>>()?;

        if size.len() != 2 {
            return Err("Size line must have exactly two values".into());
        }

        let (_x, y) = (size[0], size[1]);
        let row_count = 2 * y - 1;

        let mut grid: Vec<Vec<char>> = Vec::new();
        for _ in 0..row_count {
            let row_line = lines.next().ok_or("Expected more lines for graph grid")?;
            grid.push(row_line.chars().collect());
        }

        let mut vertex_map = HashMap::new();
        let mut vertex_id_counter = 0;

        for py in 0..row_count as usize {
            for px in 0..grid[py].len() {
                if py % 2 == 0 && px % 2 == 0 {
                    if let Some(&'o') = grid[py].get(px) {
                        vertex_map.insert((px, py), vertex_id_counter);
                        vertex_id_counter += 1;
                    }
                }
            }
        }

        let mut edges = Vec::new();
        for py in 0..row_count as usize {
            for px in 0..grid[py].len() {
                match grid[py].get(px) {
                    Some('-') if py % 2 == 0 && px % 2 == 1 => {
                        let v1 = vertex_map
                            .get(&(px - 1, py))
                            .ok_or("Edge missing start vertex")?;
                        let v2 = vertex_map
                            .get(&(px + 1, py))
                            .ok_or("Edge missing end vertex")?;
                        edges.push(Edge {
                            v1: *v1,
                            d1: Direction::Right,
                            v2: *v2,
                            d2: Direction::Left,
                        });
                    }
                    Some('|') if py % 2 == 1 && px % 2 == 0 => {
                        let v1 = vertex_map
                            .get(&(px, py - 1))
                            .ok_or("Edge missing top vertex")?;
                        let v2 = vertex_map
                            .get(&(px, py + 1))
                            .ok_or("Edge missing bottom vertex")?;
                        edges.push(Edge {
                            v1: *v1,
                            d1: Direction::Down,
                            v2: *v2,
                            d2: Direction::Up,
                        });
                    }
                    _ => {}
                }
            }
        }

        all_graphs.push((
            GraphR {
                n: vertex_id_counter,
                edges,
            },
            grid,
            vertex_map,
        ));
    }

    Ok(all_graphs)
}

fn main() {
    profile!(run);
}

fn run() {
    let args: Vec<String> = std::env::args().collect();
    let path = args
        .get(1)
        .map(|s| s.as_str())
        .unwrap_or("resources/transform0y.txt");

    // parse graphs along with grid and vertex mapping
    let graphs_with_grid =
        parse_graphs(path).unwrap_or_else(|e| panic!("Error parsing graphs: {}", e));

    if graphs_with_grid.len() < 2 {
        panic!("Expected at least two graphs in the input file");
    }

    let (start, grid_start, vertex_map_start) = graphs_with_grid[0].clone();
    let (end, _, _) = graphs_with_grid[1].clone();

    println!("Start graph: {:?}", start);
    println!("End graph: {:?}", end);

    start.print_dot();
    end.print_dot();

    if are_transformers(
        &start,
        &end,
        &grid_start,
        &vertex_map_start,
        &format!("{}_out.txt", path),
    ) {
        println!("The graphs are transformers of each other.");
    } else {
        println!("The graphs are NOT transformers of each other.");
    }
}

// DEBUG ONLY
use std::fmt::Write as FmtWrite;
impl GraphR {
    pub fn to_dot(&self) -> String {
        let mut out = String::new();

        writeln!(&mut out, "digraph G {{").unwrap();
        writeln!(&mut out, "    rankdir=LR;").unwrap();
        writeln!(&mut out).unwrap();

        // Nodes
        for v in 0..self.n {
            writeln!(&mut out, "    {} [label=\"{}\"];", v, v).unwrap();
        }

        writeln!(&mut out).unwrap();

        // Edges
        for e in &self.edges {
            writeln!(
                &mut out,
                "    {} -> {} [label=\"{:?} → {:?}\"];",
                e.v1, e.v2, e.d1, e.d2
            )
            .unwrap();
        }

        writeln!(&mut out, "}}").unwrap();

        out
    }

    pub fn print_dot(&self) {
        println!("{}", self.to_dot());
    }
}

fn are_transformers(
    graph1: &GraphR,
    graph2: &GraphR,
    grid: &[Vec<char>],
    vertex_map: &HashMap<(usize, usize), u32>,
    out_path: &str,
) -> bool {
    if graph1.n != graph2.n
        || graph1.degree_sequence() != graph2.degree_sequence()
        || graph1.signatures() != graph2.signatures()
    {
        write_solution(out_path, &[], grid, vertex_map);
        return false;
    }

    let sig_map1 = graph1.signature_to_node_list();
    let sig_map2 = graph2.signature_to_node_list();

    // group vertices by signature
    let mut classes: Vec<(Vec<u32>, Vec<u32>)> = Vec::new();
    for (sig, nodes1) in &sig_map1 {
        let nodes2 = match sig_map2.get(sig) {
            Some(n) => n,
            None => {
                write_solution(out_path, &[], grid, vertex_map);
                return false;
            }
        };
        if nodes1.len() != nodes2.len() {
            write_solution(out_path, &[], grid, vertex_map);
            return false;
        }
        classes.push((nodes1.clone(), nodes2.clone()));
    }

    // backtracking function to try all mappings
    fn backtrack(
        classes: &[(Vec<u32>, Vec<u32>)],
        index: usize,
        current_phi: &mut HashMap<u32, u32>,
        graph1: &GraphR,
        graph2: &GraphR,
        grid: &[Vec<char>],
        vertex_map: &HashMap<(usize, usize), u32>,
        out_path: &str,
    ) -> Option<Vec<u32>> {
        if index == classes.len() {
            // check consistency with DFS
            let adj = build_adjacency(graph1, graph2, current_phi);
            let mut x: HashMap<u32, u8> = HashMap::new();
            let mut steps: Vec<u32> = Vec::new();
            for v in 0..graph1.n {
                if !x.contains_key(&v) {
                    if !dfs(v, 0, &adj, &mut x, &mut steps) {
                        return None;
                    }
                }
            }
            return Some(steps);
        }

        let (nodes1, nodes2) = &classes[index];
        let mut used = vec![false; nodes2.len()];

        fn permute(
            nodes1: &[u32],
            nodes2: &mut Vec<u32>,
            used: &mut Vec<bool>,
            current_phi: &mut HashMap<u32, u32>,
            graph1: &GraphR,
            graph2: &GraphR,
            classes: &[(Vec<u32>, Vec<u32>)],
            idx: usize,
            grid: &[Vec<char>],
            vertex_map: &HashMap<(usize, usize), u32>,
            out_path: &str,
        ) -> Option<Vec<u32>> {
            if idx == nodes1.len() {
                return backtrack(
                    &classes[1..],
                    0,
                    current_phi,
                    graph1,
                    graph2,
                    grid,
                    vertex_map,
                    out_path,
                );
            }
            for j in 0..nodes2.len() {
                if !used[j] {
                    used[j] = true;
                    current_phi.insert(nodes1[idx], nodes2[j]);
                    if let Some(res) = permute(
                        nodes1,
                        nodes2,
                        used,
                        current_phi,
                        graph1,
                        graph2,
                        classes,
                        idx + 1,
                        grid,
                        vertex_map,
                        out_path,
                    ) {
                        return Some(res);
                    }
                    current_phi.remove(&nodes1[idx]);
                    used[j] = false;
                }
            }
            None
        }

        permute(
            nodes1,
            &mut nodes2.clone(),
            &mut used,
            current_phi,
            graph1,
            graph2,
            classes,
            0,
            grid,
            vertex_map,
            out_path,
        )
    }

    let mut current_phi = HashMap::new();
    if let Some(steps) = backtrack(
        &classes,
        0,
        &mut current_phi,
        graph1,
        graph2,
        grid,
        vertex_map,
        out_path,
    ) {
        let num_steps = steps.len();
        let mut file = File::create(out_path).unwrap();
        writeln!(file, "y").unwrap();
        writeln!(file, "{}", num_steps).unwrap();
        for &v in &steps {
            let pos = vertex_coords(v, vertex_map);
            writeln!(file, "{} {}", pos.0, pos.1).unwrap();
            writeln!(file, "{}", render_change(grid, pos)).unwrap();
            writeln!(file).unwrap();
        }
        true
    } else {
        write_solution(out_path, &[], grid, vertex_map);
        false
    }
}

/// Build adjacency list with edges labeled 0 (no flip) or 1 (must flip)
fn build_adjacency(
    graph: &GraphR,
    target: &GraphR,
    phi: &HashMap<u32, u32>,
) -> HashMap<u32, Vec<(u32, u8)>> {
    let mut adj: HashMap<u32, Vec<(u32, u8)>> = HashMap::new();

    for e in &graph.edges {
        let v1 = e.v1;
        let v2 = e.v2;

        let w1 = *phi.get(&v1).unwrap();
        let w2 = *phi.get(&v2).unwrap();

        let start_dirs = (e.d1, e.d2);

        let target_edge = target
            .edges
            .iter()
            .find(|te| (te.v1 == w1 && te.v2 == w2) || (te.v1 == w2 && te.v2 == w1))
            .expect("Edge must exist");

        let c = if (start_dirs.0 != target_edge.d1 || start_dirs.1 != target_edge.d2)
            && (start_dirs.0 != target_edge.d2 || start_dirs.1 != target_edge.d1)
        {
            1
        } else {
            0
        };

        adj.entry(v1).or_default().push((v2, c));
        adj.entry(v2).or_default().push((v1, c));
    }

    adj
}

/// DFS to check if a consistent flip assignment exists
fn dfs(
    v: u32,
    color: u8,
    adj: &HashMap<u32, Vec<(u32, u8)>>,
    x: &mut HashMap<u32, u8>,
    steps: &mut Vec<u32>,
) -> bool {
    x.insert(v, color);
    if color == 1 {
        steps.push(v);
    }

    if let Some(neighbors) = adj.get(&v) {
        for &(w, c) in neighbors {
            if let Some(&xw) = x.get(&w) {
                if xw != (color ^ c) {
                    return false;
                }
            } else {
                if !dfs(w, color ^ c, adj, x, steps) {
                    return false;
                }
            }
        }
    }

    true
}

fn render_change(grid: &[Vec<char>], v_pos: (usize, usize)) -> String {
    let mut out = String::new();
    for (y, row) in grid.iter().enumerate() {
        for (x, &c) in row.iter().enumerate() {
            if (x, y) == v_pos {
                out.push('#');
            } else {
                out.push(c);
            }
        }
        out.push('\n');
    }
    out
}

/// Converts vertex index to (x, y) in grid
fn vertex_coords(vertex: u32, vertex_map: &HashMap<(usize, usize), u32>) -> (usize, usize) {
    for (&(x, y), &v) in vertex_map {
        if v == vertex {
            return (x, y);
        }
    }
    panic!("Vertex not found in map");
}

fn write_solution(
    path: &str,
    steps: &[u32],
    grid: &[Vec<char>],
    vertex_map: &HashMap<(usize, usize), u32>,
) {
    let mut file = File::create(path).unwrap();
    if steps.is_empty() {
        writeln!(file, "n").unwrap();
        return;
    }
    writeln!(file, "y").unwrap();
    writeln!(file, "{}", steps.len()).unwrap();

    for &v in steps {
        let pos = vertex_coords(v, vertex_map);
        writeln!(file, "{} {}", pos.0, pos.1).unwrap();
        writeln!(file, "{}", render_change(grid, pos)).unwrap();
        writeln!(file).unwrap();
    }
}
