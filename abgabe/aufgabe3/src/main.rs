mod profile;

use petgraph::graph::{Graph, NodeIndex};
use petgraph::Undirected;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::str::FromStr;

#[derive(Debug, Hash, Eq, PartialEq, Clone)]
enum LocationType {
    Start,
    Goal,
    Num(u32),
}

impl FromStr for LocationType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "S" => Ok(LocationType::Start),
            "T" => Ok(LocationType::Goal),
            _ => s
                .parse::<u32>()
                .map(LocationType::Num)
                .map_err(|_| format!("Invalid location: {}", s)),
        }
    }
}

struct RouteNode {
    loc: LocationType,
    step: Option<u32>,
}

impl RouteNode {
    fn format(&self) -> String {
        match self.loc {
            LocationType::Start => "S".to_string(),
            LocationType::Goal => "T".to_string(),
            LocationType::Num(id) => {
                if let Some(s) = self.step {
                    format!("[{} {}]", id, s)
                } else {
                    id.to_string()
                }
            }
        }
    }
}

#[derive(Debug)]
struct ProblemData {
    max_time: u32,
    count_steps: u32,
    factories: Vec<(u32, u32)>,
    grouped_factories: Vec<Vec<u32>>,
    streets: Vec<(LocationType, LocationType, u32)>,
    factory_to_step: HashMap<u32, u32>,
}

fn get_data(path: &str) -> Result<ProblemData, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let mut lines = BufReader::new(file).lines().map(|l| l.unwrap());

    let max_time: u32 = lines.next().unwrap().trim().parse()?;
    let count_steps: u32 = lines.next().unwrap().trim().parse()?;
    let _counts: Vec<u32> = lines
        .next()
        .unwrap()
        .split_whitespace()
        .map(|x| x.parse().unwrap())
        .collect();
    let total_w: u32 = lines.next().unwrap().trim().parse()?;

    let mut factories = Vec::new();
    let mut factory_to_step = HashMap::new();
    let mut grouped_factories = vec![vec![]; (count_steps + 1) as usize];
    for _ in 0..total_w {
        let nums: Vec<u32> = lines
            .next()
            .unwrap()
            .split_whitespace()
            .map(|x| x.parse().unwrap())
            .collect();
        factories.push((nums[0], nums[1]));
        factory_to_step.insert(nums[0], nums[1]);
        grouped_factories[nums[1] as usize].push(nums[0]);
    }

    let count_streets: u32 = lines.next().unwrap().trim().parse()?;
    let mut streets = Vec::new();
    for _ in 0..count_streets {
        let line = lines.next().unwrap();
        let parts: Vec<&str> = line.split_whitespace().collect();
        streets.push((parts[0].parse()?, parts[1].parse()?, parts[2].parse()?));
    }

    Ok(ProblemData {
        max_time,
        count_steps,
        factories,
        grouped_factories,
        streets,
        factory_to_step,
    })
}

fn solve() {
    // --- PARSE ---
    let args: Vec<String> = std::env::args().collect();
    let export_dot = args.last().map(|s| s == "-D").unwrap_or(false);
    let path_args: Vec<&str> = if export_dot {
        args.iter()
            .skip(1)
            .take(args.len().saturating_sub(2))
            .map(|s| s.as_str())
            .collect()
    } else {
        args.iter().skip(1).map(|s| s.as_str()).collect()
    };
    let input_path = path_args.get(0).unwrap_or(&"resources/lieferung00.txt");
    let output_path = path_args
        .get(1)
        .map(|s| s.to_string())
        .unwrap_or_else(|| input_path.replace(".txt", "_out.txt"));
    let dot_path = input_path.replace(".txt", ".dot");

    // --- SOLVE ---
    let data = get_data(input_path).unwrap();
    let mut graph = Graph::<LocationType, i32, Undirected>::new_undirected();
    let mut loc_to_idx = HashMap::new();

    for (f, t, w) in &data.streets {
        let f_idx = *loc_to_idx
            .entry(f.clone())
            .or_insert_with(|| graph.add_node(f.clone()));
        let t_idx = *loc_to_idx
            .entry(t.clone())
            .or_insert_with(|| graph.add_node(t.clone()));
        graph.add_edge(f_idx, t_idx, *w as i32);
    }
    loc_to_idx
        .entry(LocationType::Start)
        .or_insert_with(|| graph.add_node(LocationType::Start));
    loc_to_idx
        .entry(LocationType::Goal)
        .or_insert_with(|| graph.add_node(LocationType::Goal));

    let mut all_dists = HashMap::new();
    let mut pois = vec![LocationType::Start];
    for (id, _) in &data.factories {
        pois.push(LocationType::Num(*id));
    }

    for start in pois {
        let start_idx = loc_to_idx[&start];
        let res = petgraph::algo::dijkstra(&graph, start_idx, None, |e| *e.weight());
        let mut map = HashMap::new();
        for (idx, d) in res {
            map.insert(graph[idx].clone(), d);
        }
        all_dists.insert(start, map);
    }

    const INF: i32 = 1_000_000;
    let (mut s_val, mut next_best, mut r_val, mut backup_node) = (
        HashMap::new(),
        HashMap::new(),
        HashMap::new(),
        HashMap::new(),
    );

    for step in (1..=data.count_steps).rev() {
        for &u in &data.grouped_factories[step as usize] {
            let u_loc = LocationType::Num(u);
            if step == data.count_steps {
                let d = *all_dists[&u_loc].get(&LocationType::Goal).unwrap_or(&INF);
                s_val.insert(u, d);
                next_best.insert(u, LocationType::Goal);
            } else {
                let (mut mv, mut bw) = (INF, LocationType::Goal);
                for &w in &data.grouped_factories[(step + 1) as usize] {
                    let d =
                        *all_dists[&u_loc].get(&LocationType::Num(w)).unwrap_or(&INF) + s_val[&w];
                    if d < mv {
                        mv = d;
                        bw = LocationType::Num(w);
                    }
                }
                s_val.insert(u, mv);
                next_best.insert(u, bw);
            }
        }
        for &v in &data.grouped_factories[step as usize] {
            let (mut mr, mut bb) = (INF, 0);
            for &u in &data.grouped_factories[step as usize] {
                if u == v {
                    continue;
                }
                let d = *all_dists[&LocationType::Num(v)]
                    .get(&LocationType::Num(u))
                    .unwrap_or(&INF)
                    + s_val[&u];
                if d < mr {
                    mr = d;
                    bb = u;
                }
            }
            r_val.insert(v, mr);
            backup_node.insert(v, bb);
        }
    }

    let (mut a_val, mut prev_secure) = (HashMap::new(), HashMap::new());
    a_val.insert(LocationType::Start, 0);

    for step in 1..=data.count_steps {
        for &v in &data.grouped_factories[step as usize] {
            let v_loc = LocationType::Num(v);
            let (mut ba, mut bp) = (INF, LocationType::Start);
            let prevs = if step == 1 {
                vec![LocationType::Start]
            } else {
                data.grouped_factories[(step - 1) as usize]
                    .iter()
                    .map(|&id| LocationType::Num(id))
                    .collect()
            };
            for u_l in prevs {
                if let Some(&au) = a_val.get(&u_l) {
                    let d = *all_dists[&u_l].get(&v_loc).unwrap_or(&INF);
                    if au + d + r_val[&v] <= data.max_time as i32 {
                        if au + d < ba {
                            ba = au + d;
                            bp = u_l;
                        }
                    }
                }
            }
            if ba < INF {
                a_val.insert(v_loc.clone(), ba);
                prev_secure.insert(v_loc, bp);
            }
        }
    }

    let (mut final_a, mut final_p) = (INF, LocationType::Start);
    for &lv in data.grouped_factories.last().unwrap() {
        let v_l = LocationType::Num(lv);
        if let Some(&av) = a_val.get(&v_l) {
            let d = *all_dists[&v_l].get(&LocationType::Goal).unwrap_or(&INF);
            if av + d < final_a {
                final_a = av + d;
                final_p = v_l;
            }
        }
    }

    // --- OUTPUT ---
    let mut out = File::create(output_path).unwrap();

    if !(final_a <= data.max_time as i32) {
        writeln!(out, "UNMOEGLICH").unwrap();
    } else {
        writeln!(out, "MOEGLICH").unwrap();
        let mut main_route_raw = vec![LocationType::Goal];
        let mut curr = final_p.clone();
        while curr != LocationType::Start {
            main_route_raw.push(curr.clone());
            curr = prev_secure[&curr].clone();
        }
        main_route_raw.push(LocationType::Start);
        main_route_raw.reverse();

        let mut max_dur = final_a;
        for loc in &main_route_raw {
            if let LocationType::Num(id) = loc {
                let d_to_v = if *loc == main_route_raw[1] {
                    all_dists[&LocationType::Start][loc]
                } else {
                    a_val[loc]
                };
                max_dur = max_dur.max(d_to_v + r_val[id]);
            }
        }
        writeln!(out, "{}\n\n{}", max_dur, final_a).unwrap();

        let main_fmt: Vec<_> = main_route_raw
            .iter()
            .map(|l| RouteNode {
                loc: l.clone(),
                step: if let LocationType::Num(id) = l {
                    data.factory_to_step.get(id).cloned()
                } else {
                    None
                },
            })
            .collect();

        writeln!(
            out,
            "{}",
            main_fmt
                .iter()
                .map(|n| n.format())
                .collect::<Vec<_>>()
                .join(" ")
        )
        .unwrap();

        let mut alt_routes_storage = HashMap::new();

        for node in &main_fmt {
            if let LocationType::Num(id) = node.loc {
                let d_to_v = if node.loc == main_route_raw[1] {
                    all_dists[&LocationType::Start][&node.loc]
                } else {
                    a_val[&node.loc]
                };
                writeln!(out, "{}", d_to_v + r_val[&id]).unwrap();

                let mut alt_p_raw = vec![node.loc.clone()];
                let mut alt_p_fmt = vec![id.to_string()];

                let (mut c_loc, mut t_loc, mut c_step) = (
                    node.loc.clone(),
                    LocationType::Num(backup_node[&id]),
                    node.step,
                );

                while c_loc != LocationType::Goal {
                    if let Some((_, path)) = petgraph::algo::astar(
                        &graph,
                        loc_to_idx[&c_loc],
                        |n| n == loc_to_idx[&t_loc],
                        |e| *e.weight(),
                        |_| 0,
                    ) {
                        for &idx in path.iter().skip(1) {
                            let loc = graph[idx].clone();
                            let s = if loc == t_loc && t_loc != LocationType::Goal {
                                c_step
                            } else {
                                None
                            };
                            alt_p_fmt.push(
                                RouteNode {
                                    loc: loc.clone(),
                                    step: s,
                                }
                                .format(),
                            );
                            alt_p_raw.push(loc.clone());
                            c_loc = loc;
                        }
                    }
                    if c_loc != LocationType::Goal {
                        if let LocationType::Num(cid) = c_loc {
                            t_loc = next_best[&cid].clone();
                            if let LocationType::Num(tid) = t_loc {
                                c_step = data.factory_to_step.get(&tid).cloned();
                            }
                        } else {
                            t_loc = LocationType::Goal;
                        }
                    }
                }
                writeln!(out, "{}", alt_p_fmt.join(" ")).unwrap();
                if export_dot {
                    alt_routes_storage.insert(id, alt_p_raw);
                }
            }
        }

        // --- DOT ---
        if export_dot {
            generate_dot_file(
                &dot_path,
                &graph,
                &data,
                &loc_to_idx,
                &main_route_raw,
                &alt_routes_storage,
            );
        }
    }
}

fn generate_dot_file(
    path: &str,
    graph: &Graph<LocationType, i32, Undirected>,
    data: &ProblemData,
    loc_to_idx: &HashMap<LocationType, NodeIndex>,
    main_route: &[LocationType],
    alt_routes: &HashMap<u32, Vec<LocationType>>,
) {
    let mut f = File::create(path).expect("Could not create DOT file");

    writeln!(f, "digraph G {{").unwrap();
    writeln!(
        f,
        "  splines=true; overlap=false; rankdir=LR; bgcolor=\"#2e2e2e\";"
    )
    .unwrap();
    writeln!(
        f,
        "  node [fontname=\"Arial\", shape=circle, style=filled];"
    )
    .unwrap();
    writeln!(f, "  edge [fontname=\"Arial\", fontsize=10];").unwrap();
    let colors = [
        "cyan",
        "green",
        "yellow",
        "orange",
        "magenta",
        "purple",
        "lightblue",
        "lime",
        "gold",
        "pink",
    ];
    for node_idx in graph.node_indices() {
        let loc = &graph[node_idx];
        let step = match loc {
            LocationType::Num(id) => data.factory_to_step.get(id).cloned(),
            _ => None,
        };
        let lbl = RouteNode {
            loc: loc.clone(),
            step,
        }
        .format();
        let color = match loc {
            LocationType::Start => "green",
            LocationType::Goal => "lightblue",
            LocationType::Num(_) => "red",
        };
        writeln!(
            f,
            "  n{} [label=\"{}\", fillcolor={}];",
            node_idx.index(),
            lbl,
            color
        )
        .unwrap();
    }
    for edge in graph.edge_indices() {
        let (u, v) = graph.edge_endpoints(edge).unwrap();
        writeln!(f, "  n{} -> n{} [dir=none, color=\"#444444\", penwidth=1, label=\"{}\", fontcolor=\"#777777\", constraint=false];",
                 u.index(), v.index(), graph[edge]).unwrap();
    }
    for (i, window) in main_route.windows(2).enumerate() {
        let u = loc_to_idx[&window[0]].index();
        let v = loc_to_idx[&window[1]].index();
        writeln!(
            f,
            "  n{} -> n{} [color=red, penwidth=4, label=\".{}\", fontcolor=red];",
            u,
            v,
            i + 1
        )
        .unwrap();
    }
    let mut sorted_keys: Vec<_> = alt_routes.keys().collect();
    sorted_keys.sort();

    for (color_idx, &&id) in sorted_keys.iter().enumerate() {
        let route_color = colors[color_idx % colors.len()];
        let route = &alt_routes[&id];

        for (step_idx, window) in route.windows(2).enumerate() {
            let u = loc_to_idx[&window[0]].index();
            let v = loc_to_idx[&window[1]].index();
            writeln!(
                f,
                "  n{} -> n{} [color={}, penwidth=2, style=dashed, label=\"{}.{}\", fontcolor={}];",
                u,
                v,
                route_color,
                id,
                step_idx + 1,
                route_color
            )
            .unwrap();
        }
    }

    writeln!(f, "}}").unwrap();
}

fn main() {
    profile!(solve);
}
