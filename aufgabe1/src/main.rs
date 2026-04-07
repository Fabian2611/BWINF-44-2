use petgraph::graph::{NodeIndex, UnGraph};
use petgraph::prelude::EdgeRef;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::cmp::min;
use std::time::Instant;

#[derive(PartialEq, Eq, Debug, Copy, Clone, Hash, PartialOrd, Ord)]
enum Axis { Horizontal, Vertical }

#[derive(PartialEq, Eq, Debug, Copy, Clone, Hash, PartialOrd, Ord)]
struct Pin {
    axis: Axis,
    sign: i8, // 1 oder -1
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum BCNode {
    ArticulationPoint(NodeIndex),
    // Ein Block wird durch seine internen Kanten-Vektoren relativ zum Eingang definiert
    Block {
        edges: Vec<(i32, i32)>, // (dx, dy) Paare der internen Stifte
    },
}

type FigureGraph = UnGraph<(), Pin>;
type BlockCutTree = UnGraph<BCNode, Pin>;

struct BiconnectedResult {
    articulation_points: HashSet<NodeIndex>,
    blocks: Vec<Vec<(NodeIndex, NodeIndex, Pin)>>,
}

impl BiconnectedResult {
    fn from_graph(g: &FigureGraph) -> Self {
        let mut disc = HashMap::new();
        let mut low = HashMap::new();
        let mut time = 0;
        let mut articulation_points = HashSet::new();
        let mut edge_stack = Vec::new();
        let mut blocks_edges = Vec::new();
        if let Some(start) = g.node_indices().next() {
            dfs_bc(start, None, g, &mut disc, &mut low, &mut time, &mut edge_stack, &mut blocks_edges, &mut articulation_points);
        }
        BiconnectedResult { articulation_points, blocks: blocks_edges }
    }
}

fn dfs_bc(u: NodeIndex, p: Option<NodeIndex>, g: &FigureGraph, d: &mut HashMap<NodeIndex, u32>, l: &mut HashMap<NodeIndex, u32>, t: &mut u32, st: &mut Vec<(NodeIndex, NodeIndex, Pin)>, bl: &mut Vec<Vec<(NodeIndex, NodeIndex, Pin)>>, ap: &mut HashSet<NodeIndex>) {
    *t += 1; d.insert(u, *t); l.insert(u, *t);
    let mut children = 0;
    for edge in g.edges(u) {
        let v = edge.target();
        if Some(v) == p { continue; }
        let pin = *edge.weight();
        if let Some(&dv) = d.get(&v) {
            l.insert(u, min(*l.get(&u).unwrap(), dv));
            if dv < *d.get(&u).unwrap() { st.push((u, v, pin)); }
        } else {
            children += 1; st.push((u, v, pin));
            dfs_bc(v, Some(u), g, d, l, t, st, bl, ap);
            let lv = *l.get(&v).unwrap();
            l.insert(u, min(*l.get(&u).unwrap(), lv));
            if lv >= *d.get(&u).unwrap() {
                if p.is_some() { ap.insert(u); }
                let mut curr = Vec::new();
                while let Some(e) = st.pop() {
                    curr.push(e);
                    if (e.0 == u && e.1 == v) || (e.0 == v && e.1 == u) { break; }
                }
                bl.push(curr);
            }
        }
    }
    if p.is_none() && children > 1 { ap.insert(u); }
}

fn build_bc_tree(_: &FigureGraph, res: &BiconnectedResult) -> BlockCutTree {
    let mut tree = BlockCutTree::default();
    let mut art_map = HashMap::new();
    for &art in &res.articulation_points {
        art_map.insert(art, tree.add_node(BCNode::ArticulationPoint(art)));
    }
    for block_edges in &res.blocks {
        let mut internal_vecs = Vec::new();
        for e in block_edges {
            let dx = if e.2.axis == Axis::Horizontal { e.2.sign as i32 } else { 0 };
            let dy = if e.2.axis == Axis::Vertical { e.2.sign as i32 } else { 0 };
            internal_vecs.push((dx, dy));
        }
        let b_idx = tree.add_node(BCNode::Block { edges: internal_vecs });
        let mut nodes = HashSet::new();
        for e in block_edges { nodes.insert(e.0); nodes.insert(e.1); }
        for &n in &nodes {
            if let Some(&a_idx) = art_map.get(&n) {
                let e = block_edges.iter().find(|e| e.0 == n || e.1 == n).unwrap();
                tree.add_edge(b_idx, a_idx, e.2);
            }
        }
    }
    tree
}

fn find_centers(t: &BlockCutTree) -> Vec<NodeIndex> {
    let cnt = t.node_count();
    if cnt <= 2 { return t.node_indices().collect(); }
    let mut deg: HashMap<_, _> = t.node_indices().map(|v| (v, t.neighbors(v).count())).collect();
    let mut leaves: Vec<_> = deg.iter().filter(|&(_, &d)| d == 1).map(|(&v, _)| v).collect();
    let mut rem = 0;
    while rem + leaves.len() < cnt {
        rem += leaves.len();
        let mut next = Vec::new();
        for l in leaves {
            for n in t.neighbors(l) {
                if let Some(d) = deg.get_mut(&n) { *d -= 1; if *d == 1 { next.push(n); } }
            }
            deg.remove(&l);
        }
        leaves = next;
    }
    deg.keys().cloned().collect()
}

fn check_transformable(u: NodeIndex, pu: Option<NodeIndex>, t_a: &BlockCutTree, v: NodeIndex, pv: Option<NodeIndex>, t_b: &BlockCutTree) -> bool {
    let configs = [(1, 1), (1, -1), (-1, 1), (-1, -1)];
    configs.iter().any(|&(xf, yf)| match_recursive(u, pu, t_a, v, pv, t_b, xf, yf))
}

fn match_recursive(u: NodeIndex, pu: Option<NodeIndex>, t_a: &BlockCutTree, v: NodeIndex, pv: Option<NodeIndex>, t_b: &BlockCutTree, xf: i8, yf: i8) -> bool {
    match (&t_a[u], &t_b[v]) {
        (BCNode::ArticulationPoint(_), BCNode::ArticulationPoint(_)) => {}
        (BCNode::Block { edges: e1 }, BCNode::Block { edges: e2 }) => {
            if e1.len() != e2.len() { return false; }
            let mut te1: Vec<_> = e1.iter().map(|&(dx, dy)| (dx * xf as i32, dy * yf as i32)).collect();
            let mut e2c = e2.clone(); te1.sort(); e2c.sort();
            if te1 != e2c { return false; }
        }
        _ => return false,
    }
    let mut ch_a: Vec<_> = t_a.edges(u).filter(|e| Some(e.target()) != pu).collect();
    let mut ch_b: Vec<_> = t_b.edges(v).filter(|e| Some(e.target()) != pv).collect();
    if ch_a.len() != ch_b.len() { return false; }

    let mut matched = vec![false; ch_b.len()];
    for ea in &ch_a {
        let pin = ea.weight();
        let tp = Pin { axis: pin.axis, sign: if pin.axis == Axis::Horizontal { pin.sign * xf } else { pin.sign * yf } };
        let mut found = false;
        for (i, eb) in ch_b.iter().enumerate() {
            if !matched[i] && *eb.weight() == tp {
                if match_recursive(ea.target(), Some(u), t_a, eb.target(), Some(v), t_b, xf, yf) {
                    matched[i] = true; found = true; break;
                }
            }
        }
        if !found { return false; }
    }
    true
}

fn parse_grid(path: &str) -> Vec<FigureGraph> {
    let s = fs::read_to_string(path).unwrap();
    let mut lines = s.lines().peekable();
    let mut gs = Vec::new();
    while let Some(l) = lines.next() {
        if l.trim().is_empty() { continue; }
        let d: Vec<usize> = l.split_whitespace().map(|x| x.parse().unwrap()).collect();
        let rows = 2 * d[1] - 1;
        let grid: Vec<Vec<char>> = (0..rows).map(|_| lines.next().unwrap().chars().collect()).collect();
        let mut g = FigureGraph::default();
        let mut vm = HashMap::new();
        for y in (0..rows).step_by(2) {
            for x in (0..grid[y].len()).step_by(2) {
                if grid[y][x] == 'o' { vm.insert((x, y), g.add_node(())); }
            }
        }
        for y in 0..rows {
            for x in 0..grid[y].len() {
                if grid[y][x] == '-' {
                    // WICHTIG: Richtung wird hier relativ zur Gitterposition definiert
                    g.add_edge(vm[&(x-1, y)], vm[&(x+1, y)], Pin { axis: Axis::Horizontal, sign: 1 });
                } else if grid[y][x] == '|' {
                    g.add_edge(vm[&(x, y-1)], vm[&(x, y+1)], Pin { axis: Axis::Vertical, sign: 1 });
                }
            }
        }
        gs.push(g);
    }
    gs
}

fn main() {
    let graphs = parse_grid("resources/transform03.txt");
    if graphs.len() < 2 { return; }
    let bca = build_bc_tree(&graphs[0], &BiconnectedResult::from_graph(&graphs[0]));
    let bcb = build_bc_tree(&graphs[1], &BiconnectedResult::from_graph(&graphs[1]));
    let (ca, cb) = (find_centers(&bca), find_centers(&bcb));
    let mut ok = false;
    for &ra in &ca {
        for &rb in &cb {
            if check_transformable(ra, None, &bca, rb, None, &bcb) { ok = true; break; }
        }
        if ok { break; }
    }
    println!("{}", if ok { "Transformable!" } else { "Not transformable!" });
}
